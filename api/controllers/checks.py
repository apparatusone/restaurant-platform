from sqlalchemy.orm import Session, joinedload
from sqlalchemy import func
from fastapi import HTTPException, status
from datetime import datetime
from decimal import Decimal
from ..models.checks import Check, CheckStatus
from ..models.orders import Order
from ..models.order_items import OrderItem
from ..models.table_sessions import TableSession
from ..schemas.checks import CheckCreate, CheckUpdate
from ..utils.errors import (
    raise_not_found,
    raise_validation_error,
    raise_status_transition_error
)


def create(db: Session, request: CheckCreate):
    # validate session for non-virtual checks
    if not request.is_virtual and request.session_id:
        session = db.query(TableSession).filter(TableSession.id == request.session_id).first()
        if not session:
            raise_not_found("Session", request.session_id)
        
        if session.closed_at:
            raise_validation_error("Cannot create check for closed session")
    elif not request.is_virtual and not request.session_id:
        raise_validation_error("Non-virtual checks must have a session_id")
    elif request.is_virtual and request.session_id:
        raise_validation_error("Virtual checks cannot have a session_id")
    
    new_check = Check(
        session_id=request.session_id,
        is_virtual=request.is_virtual,
        subtotal=request.subtotal or Decimal('0.00'),
        tax_amount=request.tax_amount or Decimal('0.00'),
        tip_amount=request.tip_amount or Decimal('0.00'),
        total_amount=request.total_amount or Decimal('0.00')
    )
    
    db.add(new_check)
    db.commit()
    db.refresh(new_check)
    return new_check


def read_all(db: Session):
    """Get all checks with session info - handles both virtual and table checks"""
    return db.query(Check).options(
        joinedload(Check.session),
        joinedload(Check.orders)
    ).all()


def read_one(db: Session, check_id: int):
    """Get single check with full details - works for both virtual and table checks"""
    check = db.query(Check).options(
        joinedload(Check.session),
        joinedload(Check.orders).joinedload(Order.order_items),
        joinedload(Check.payments)
    ).filter(Check.id == check_id).first()
    if not check:
        raise_not_found("Check", check_id)
    return check


def read_by_session(db: Session, session_id: int):
    """Get all checks for a specific session"""
    return db.query(Check).filter(Check.session_id == session_id).all()


def update(db: Session, check_id: int, request: CheckUpdate):
    check = db.query(Check).filter(Check.id == check_id).first()
    if not check:
        raise_not_found("Check", check_id)
    
    update_data = request.model_dump(exclude_unset=True)
    for field, value in update_data.items():
        setattr(check, field, value)
    
    # timestamps
    if request.status == CheckStatus.SENT and not check.submitted_at:
        check.submitted_at = datetime.utcnow()
    elif request.status == CheckStatus.PAID and not check.paid_at:
        check.paid_at = datetime.utcnow()
    
    db.commit()
    db.refresh(check)
    return check


# change check status and submitted time
def submit_check(db: Session, check_id: int):
    """Submit check for payment - works for both virtual and table checks"""
    check = db.query(Check).filter(Check.id == check_id).first()
    if not check:
        raise_not_found("Check", check_id)
    
    if check.status != CheckStatus.OPEN:
        raise_validation_error(f"Check is already {check.status.value}")
    
    # update totals before submitting
    update_check_totals(db, check_id)
    
    check.status = CheckStatus.SENT
    check.submitted_at = datetime.utcnow()
    
    db.commit()
    db.refresh(check)
    return check


def process_payment(db: Session, check_id: int, tip_amount: Decimal = None):
    """Process payment for check - works for both virtual and table checks"""
    check = db.query(Check).filter(Check.id == check_id).first()
    if not check:
        raise_not_found("Check", check_id)
    
    if check.status not in [CheckStatus.SENT, CheckStatus.READY]:
        raise_validation_error(f"Check must be sent before payment. Current status: {check.status.value}")
    
    # update totals to ensure accuracy
    update_check_totals(db, check_id)
    
    if tip_amount is not None:
        check.tip_amount = tip_amount
        check.total_amount = check.subtotal + check.tax_amount + check.tip_amount
    
    check.status = CheckStatus.PAID
    check.paid_at = datetime.utcnow()
    
    db.commit()
    db.refresh(check)
    return check


def delete(db: Session, check_id: int):
    check = db.query(Check).filter(Check.id == check_id).first()
    if not check:
        raise_not_found("Check", check_id)

    if check.status in [CheckStatus.SENT, CheckStatus.READY, CheckStatus.PAID, CheckStatus.CLOSED]:
        raise_validation_error("Cannot delete submitted or paid check")

    # Only allow delete if no menu items in the order
    order = check.order
    if order:
        item_count = db.query(order.order_items).count()
        if item_count > 0:
            raise_validation_error("Cannot delete check with menu items in its order")

    db.delete(check)
    db.commit()
    return {"message": f"Check {check_id} deleted successfully"}


def get_open_checks(db: Session, include_virtual: bool = True, include_table: bool = True):
    """Get all open checks - unified for virtual and table checks"""
    query = db.query(Check).filter(Check.status == CheckStatus.OPEN).options(
        joinedload(Check.session),
        joinedload(Check.orders)
    )
    
    if include_virtual and not include_table:
        query = query.filter(Check.is_virtual == True)
    elif include_table and not include_virtual:
        query = query.filter(Check.is_virtual == False)
    
    return query.all()


def get_pending_checks(db: Session, include_virtual: bool = True, include_table: bool = True):
    """Get all checks pending payment"""
    query = db.query(Check).filter(
        Check.status.in_([CheckStatus.SENT, CheckStatus.READY])
    ).options(
        joinedload(Check.session),
        joinedload(Check.orders)
    )
    
    if include_virtual and not include_table:
        query = query.filter(Check.is_virtual == True)
    elif include_table and not include_virtual:
        query = query.filter(Check.is_virtual == False)
    
    return query.all()


def get_virtual_checks(db: Session, status: CheckStatus = None):
    """Get virtual checks with optional status filter"""
    query = db.query(Check).filter(Check.is_virtual == True).options(
        joinedload(Check.orders).joinedload(Order.order_items)
    )
    
    if status:
        query = query.filter(Check.status == status)
    
    return query.all()


def get_table_checks(db: Session, status: CheckStatus = None):
    """Get a table's checks with optional status filter"""
    query = db.query(Check).filter(Check.is_virtual == False).options(
        joinedload(Check.session),
        joinedload(Check.orders).joinedload(Order.order_items)
    )
    
    if status:
        query = query.filter(Check.status == status)
    
    return query.all()


def calculate_check_total(db: Session, check_id: int):
    """Calculate check total from associated orders - works for both virtual and table checks"""
    # get sum of all order details for orders in this check
    subtotal = db.query(func.sum(OrderItem.line_total)).join(Order).filter(
        Order.check_id == check_id
    ).scalar() or Decimal('0.00')
    
    # for now, use simple tax calculation (8.5%)
    #TODO: replace with config tax rate
    tax_rate = Decimal('0.085')
    tax_amount = subtotal * tax_rate
    
    return {
        'subtotal': subtotal,
        'tax_amount': tax_amount,
        'total_amount': subtotal + tax_amount
    }


def update_check_totals(db: Session, check_id: int):
    """Update check totals based on associated orders - unified for virtual and table checks"""
    check = db.query(Check).filter(Check.id == check_id).first()
    if not check:
        raise_not_found("Check", check_id)
    
    totals = calculate_check_total(db, check_id)
    
    check.subtotal = totals['subtotal']
    check.tax_amount = totals['tax_amount']
    # preserve existing tip amount
    check.total_amount = totals['total_amount'] + (check.tip_amount or Decimal('0.00'))
    
    db.commit()
    db.refresh(check)
    return check


def get_checks_with_orders(db: Session, include_virtual: bool = True, include_table: bool = True):
    """Get checks with their associated orders"""
    query = db.query(Check).options(
        joinedload(Check.orders).joinedload(Order.order_items),
        joinedload(Check.session)
    )
    
    if include_virtual and not include_table:
        query = query.filter(Check.is_virtual == True)
    elif include_table and not include_virtual:
        query = query.filter(Check.is_virtual == False)
    # if both are True, no filter needed (get all checks)
    
    return query.all()


def get_check_summary(db: Session, check_id: int):
    """Get comprehensive check summary - works for both virtual and table checks"""
    check = db.query(Check).options(
        joinedload(Check.orders).joinedload(Order.order_items).joinedload(OrderItem.menu_item),
        joinedload(Check.session),
        joinedload(Check.payments)
    ).filter(Check.id == check_id).first()
    
    if not check:
        raise_not_found("Check", check_id)
    
    # calculate current totals from orders
    calculated_totals = calculate_check_total(db, check_id)
    
    # calculate payments made
    total_payments = sum(payment.amount for payment in check.payments) if check.payments else Decimal('0.00')
    
    return {
        'check': check,
        'calculated_totals': calculated_totals,
        'total_payments': total_payments,
        'balance_due': (calculated_totals['total_amount'] + (check.tip_amount or Decimal('0.00'))) - total_payments,
        'order_count': len(check.orders),
        'is_virtual': check.is_virtual
    }


def update_check_status(db: Session, check_id: int, new_status: CheckStatus):
    """Update check status with proper validation - works for both virtual and table checks"""
    check = db.query(Check).filter(Check.id == check_id).first()
    if not check:
        raise_not_found("Check", check_id)
    
    # validate status transitions
    valid_transitions = {
        CheckStatus.OPEN: [CheckStatus.SENT],
        CheckStatus.SENT: [CheckStatus.READY, CheckStatus.PAID],
        CheckStatus.READY: [CheckStatus.PAID],
        CheckStatus.PAID: [CheckStatus.CLOSED],
        CheckStatus.CLOSED: []  # terminal state
    }
    
    if new_status not in valid_transitions.get(check.status, []):
        valid_next_states = [s.value for s in valid_transitions.get(check.status, [])]
        raise_status_transition_error(
            check.status.value,
            new_status.value,
            valid_next_states
        )
    
    # update totals before status change if moving to sent or paid
    if new_status in [CheckStatus.SENT, CheckStatus.PAID]:
        update_check_totals(db, check_id)
    
    # set timestamps based on status
    if new_status == CheckStatus.SENT and not check.submitted_at:
        check.submitted_at = datetime.utcnow()
    elif new_status == CheckStatus.PAID and not check.paid_at:
        check.paid_at = datetime.utcnow()
    
    check.status = new_status
    db.commit()
    db.refresh(check)
    return check


def get_checks_by_order_type(db: Session, order_type: str = None):
    """Get checks filtered by associated order types - unified querying"""
    query = db.query(Check).options(
        joinedload(Check.orders),
        joinedload(Check.session)
    )
    
    if order_type:
        from ..models.orders import OrderType
        query = query.join(Order).filter(Order.order_type == OrderType(order_type))
    
    return query.all()


def send_check_to_kitchen(db: Session, check_id: int):
    """Send all unsent items in a check to kitchen"""
    from ..models.order_items import OrderItem, OrderItemStatus
    
    check = db.query(Check).filter(Check.id == check_id).first()
    if not check:
        raise_not_found("Check", check_id)
    
    unsent_items = db.query(OrderItem).join(Order).filter(
        Order.check_id == check_id,
        OrderItem.status == OrderItemStatus.UNSENT
    ).all()
    
    if not unsent_items:
        raise_validation_error("No unsent items found in this check")
    
    # mark all unsent items as sent
    for item in unsent_items:
        item.status = OrderItemStatus.SENT
    
    # if check is open mark status as sent
    if check.status == CheckStatus.OPEN:
        check.status = CheckStatus.SENT
        check.submitted_at = datetime.utcnow()
    
    db.commit()
    db.refresh(check)
    return check


def create_payment_for_check(db: Session, check_id: int, request):
    """Create payment for a check with business rule validation"""
    from ..models.payment_method import Payment, PaymentType, PaymentStatus
    
    check = db.query(Check).filter(Check.id == check_id).first()
    if not check:
        raise_not_found("Check", check_id)
    
    if check.status != CheckStatus.READY:
        raise_validation_error(f"Cannot process payment for check with status '{check.status.value}'. Check must be ready.")
    
    # validate amount
    if request.amount <= 0:
        raise_validation_error("Payment amount must be greater than 0")
    
    # check if it's already been paid
    existing_payments = db.query(Payment).filter(
        Payment.check_id == check_id,
        Payment.status.in_([PaymentStatus.COMPLETED, PaymentStatus.PENDING])
    ).all()
    
    total_existing = sum(payment.amount for payment in existing_payments)
    
    remaining_balance = check.total_amount - total_existing
    if request.amount > remaining_balance:
        raise_validation_error(f"Payment amount ({request.amount}) exceeds remaining balance ({remaining_balance})")
    
    payment_status = PaymentStatus.COMPLETED if request.payment_type == PaymentType.CASH else PaymentStatus.COMPLETED
    
    new_payment = Payment(
        check_id=check_id,
        amount=request.amount,
        payment_type=request.payment_type,
        status=payment_status,
        card_number=request.card_number
    )
    
    db.add(new_payment)
    
    # calc remaining
    total_payments_after = total_existing + request.amount
    if total_payments_after >= check.total_amount:
        check.status = CheckStatus.PAID
        check.paid_at = datetime.utcnow()
    
    db.commit()
    db.refresh(check)
    return check


def close_check(db: Session, check_id: int):
    """Close a paid check"""
    check = db.query(Check).filter(Check.id == check_id).first()
    if not check:
        raise_not_found("Check", check_id)
    
    # onlya allow closing paid checks
    if check.status != CheckStatus.PAID:
        raise_validation_error(f"Cannot close check with status '{check.status.value}'. Check must be paid before closing.")
    
    check.status = CheckStatus.CLOSED
    check.updated_at = datetime.utcnow()
    
    db.commit()
    db.refresh(check)
    return check


