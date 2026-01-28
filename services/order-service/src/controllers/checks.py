from sqlalchemy.orm import Session, joinedload
from sqlalchemy import func
from fastapi import HTTPException, status
from datetime import datetime, timezone
from decimal import Decimal
from ..models.checks import Check, CheckStatus
from shared.models.check_items import CheckItem, CheckItemStatus
from ..schemas.checks import CheckCreate, CheckUpdate
from ..utils.errors import (
    raise_not_found,
    raise_validation_error,
    raise_status_transition_error
)
from ..clients import restaurant_client, payment_client
import asyncio
import logging

logger = logging.getLogger(__name__)


async def create(db: Session, request: CheckCreate):
    # validate seating for non-virtual checks
    if not request.is_virtual and request.seating_id:
        # Call restaurant-service to validate seating
        try:
            response = await restaurant_client.get(f"/seatings/{request.seating_id}")
            seating = response.json()
            if not seating:
                raise_not_found("Seating", request.seating_id)
            
            if seating.get('closed_at'):
                raise_validation_error("Cannot create check for closed seating")
        except Exception as e:
            logger.error(f"Failed to validate seating: {e}")
            raise_validation_error(f"Failed to validate seating: {str(e)}")
    elif not request.is_virtual and not request.seating_id:
        raise_validation_error("Non-virtual checks must have a seating_id")
    elif request.is_virtual and request.seating_id:
        raise_validation_error("Virtual checks cannot have a seating_id")
    
    new_check = Check(
        seating_id=request.seating_id,
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
    """Get all checks with seating info - handles both virtual and table checks"""
    return db.query(Check).options(
        joinedload(Check.check_items)
    ).all()


def read_one(db: Session, check_id: int):
    """Get single check with full details - works for both virtual and table checks"""
    check = db.query(Check).options(
        joinedload(Check.check_items)
    ).filter(Check.id == check_id).first()
    if not check:
        raise_not_found("Check", check_id)
    return check


def read_by_seating(db: Session, seating_id: int):
    """Get all checks for a specific seating"""
    return db.query(Check).filter(Check.seating_id == seating_id).all()


def update(db: Session, check_id: int, request: CheckUpdate):
    check = db.query(Check).filter(Check.id == check_id).first()
    if not check:
        raise_not_found("Check", check_id)
    
    update_data = request.model_dump(exclude_unset=True)
    for field, value in update_data.items():
        setattr(check, field, value)
    
    # timestamps
    if request.status == CheckStatus.SENT and not check.submitted_at:
        check.submitted_at = datetime.now(timezone.utc)
    elif request.status == CheckStatus.PAID and not check.paid_at:
        check.paid_at = datetime.now(timezone.utc)
    
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
    check.submitted_at = datetime.now(timezone.utc)
    
    db.commit()
    db.refresh(check)
    
    # Notify restaurant-service kitchen queue (async call)
    # Note: This is a write operation to the kitchen queue, so we call the owner service
    if check.restaurant_id:
        try:
            asyncio.create_task(_notify_kitchen_queue(check.restaurant_id, check_id))
        except Exception as e:
            logger.warning(f"Failed to notify kitchen queue for check {check_id}: {e}")
            # Don't fail the check submission if kitchen notification fails
    
    return check


def delete(db: Session, check_id: int):
    check = db.query(Check).filter(Check.id == check_id).first()
    if not check:
        raise_not_found("Check", check_id)

    if check.status in [CheckStatus.SENT, CheckStatus.READY, CheckStatus.PAID, CheckStatus.CLOSED]:
        raise_validation_error("Cannot delete submitted or paid check")

    # Only allow delete if no check items
    item_count = db.query(CheckItem).filter(CheckItem.check_id == check_id).count()
    if item_count > 0:
        raise_validation_error("Cannot delete check with check items")

    db.delete(check)
    db.commit()
    return {"message": f"Check {check_id} deleted successfully"}


def get_open_checks(db: Session, include_virtual: bool = True, include_table: bool = True):
    """Get all open checks - unified for virtual and table checks"""
    query = db.query(Check).filter(Check.status == CheckStatus.OPEN).options(
        joinedload(Check.check_items)
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
        joinedload(Check.check_items)
    )
    
    if include_virtual and not include_table:
        query = query.filter(Check.is_virtual == True)
    elif include_table and not include_virtual:
        query = query.filter(Check.is_virtual == False)
    
    return query.all()


def get_virtual_checks(db: Session, status: CheckStatus = None):
    """Get virtual checks with optional status filter"""
    query = db.query(Check).filter(Check.is_virtual == True).options(
        joinedload(Check.check_items)
    )
    
    if status:
        query = query.filter(Check.status == status)
    
    return query.all()


def get_table_checks(db: Session, status: CheckStatus = None):
    """Get a table's checks with optional status filter"""
    query = db.query(Check).filter(Check.is_virtual == False).options(
        joinedload(Check.check_items)
    )
    
    if status:
        query = query.filter(Check.status == status)
    
    return query.all()


def calculate_check_total(db: Session, check_id: int):
    """Calculate check total from check items directly - works for both virtual and table checks"""
    # get sum of all check items for this check
    subtotal = db.query(func.sum(CheckItem.total_price)).filter(
        CheckItem.check_id == check_id
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


def get_checks_with_items(db: Session, include_virtual: bool = True, include_table: bool = True):
    """Get checks with their associated check items"""
    query = db.query(Check).options(
        joinedload(Check.check_items)
    )
    
    if include_virtual and not include_table:
        query = query.filter(Check.is_virtual == True)
    elif include_table and not include_virtual:
        query = query.filter(Check.is_virtual == False)
    # if both are True, no filter needed (get all checks)
    
    return query.all()


async def get_check_summary(db: Session, check_id: int):
    """Get comprehensive check summary - works for both virtual and table checks"""
    check = db.query(Check).options(
        joinedload(Check.check_items).joinedload(CheckItem.menu_item)
    ).filter(Check.id == check_id).first()
    
    if not check:
        raise_not_found("Check", check_id)
    
    # calculate current totals from check items
    calculated_totals = calculate_check_total(db, check_id)
    
    # Fetch payments from payment-service (read operation)
    total_payments = Decimal('0.00')
    try:
        response = await payment_client.get(f"/payments/check/{check_id}")
        if response.status_code == 200:
            payments = response.json()
            total_payments = sum(Decimal(str(p.get('amount', 0))) for p in payments)
    except Exception as e:
        logger.warning(f"Failed to fetch payments from payment-service: {e}")
        # Continue without payment data
    
    return {
        'check': check,
        'calculated_totals': calculated_totals,
        'total_payments': total_payments,
        'balance_due': (calculated_totals['total_amount'] + (check.tip_amount or Decimal('0.00'))) - total_payments,
        'item_count': len(check.check_items) if check.check_items else 0,
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
        check.submitted_at = datetime.now(timezone.utc)
    elif new_status == CheckStatus.PAID and not check.paid_at:
        check.paid_at = datetime.now(timezone.utc)
    
    check.status = new_status
    db.commit()
    db.refresh(check)
    return check


def get_checks_by_type(db: Session, is_virtual: bool = None):
    """Get checks filtered by type (virtual or table)"""
    query = db.query(Check).options(
        joinedload(Check.check_items)
    )
    
    if is_virtual is not None:
        query = query.filter(Check.is_virtual == is_virtual)
    
    return query.all()


async def create_payment_for_check(db: Session, check_id: int, request):
    """Create payment for a check with business rule validation - delegates to payment-service"""
    from shared.models.check_items import CheckItemStatus
    
    check = db.query(Check).filter(Check.id == check_id).first()
    if not check:
        raise_not_found("Check", check_id)
    
    # For dine-in checks (non-virtual), require status to be READY and all items ready
    # For online orders (virtual checks), allow payment at any status after OPEN
    if not check.is_virtual:
        if check.status != CheckStatus.READY:
            raise_validation_error(f"Cannot process payment for check with status '{check.status.value}'. Check must be ready.")
        
        # Verify all check items are actually ready for dine-in
        all_items = db.query(CheckItem).filter(
            CheckItem.check_id == check_id
        ).all()
        
        if all_items:
            not_ready_items = [item for item in all_items if item.status != CheckItemStatus.READY]
            if not_ready_items:
                raise_validation_error(
                    f"Cannot process payment. {len(not_ready_items)} item(s) are not ready yet. "
                    f"All items must be ready before payment."
                )
    else:
        # For virtual checks, allow payment after items are sent (OPEN -> SENT -> payment)
        if check.status == CheckStatus.OPEN:
            raise_validation_error(f"Cannot process payment for online order with status '{check.status.value}'. Items must be sent to kitchen first.")
    
    # validate amount
    if request.amount <= 0:
        raise_validation_error("Payment amount must be greater than 0")
    
    # Call payment-service to create payment (write operation)
    # payment-service owns the Payment table, so we must call it for writes
    try:
        response = await payment_client.post(
            "/payments",
            json={
                "check_id": check_id,
                "amount": float(request.amount),
                "payment_type": request.payment_type.value if hasattr(request.payment_type, 'value') else request.payment_type,
                "card_number": request.card_number
            }
        )
        
        if response.status_code == 201 or response.status_code == 200:
            payment_data = response.json()
            logger.info(f"Payment created via payment-service: {payment_data}")
            
            # payment-service will notify us back to update check status
            # For now, we can update the check status here if payment is completed
            # Note: In production, payment-service should call us back via webhook
            
            return check
        else:
            raise_validation_error(f"Payment service returned error: {response.status_code}")
    
    except Exception as e:
        logger.error(f"Failed to create payment via payment-service: {e}")
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail=f"Payment service unavailable: {str(e)}"
        )


def close_check(db: Session, check_id: int):
    """Close a paid check"""
    check = db.query(Check).filter(Check.id == check_id).first()
    if not check:
        raise_not_found("Check", check_id)

    if check.status == CheckStatus.PAID:
        check.status = CheckStatus.CLOSED
        check.closed_at = datetime.now(timezone.utc)
        check.updated_at = datetime.now(timezone.utc)
    elif check.status == CheckStatus.OPEN:
        item_count = db.query(CheckItem).filter(CheckItem.check_id == check_id).count()
        if item_count > 0:
            raise_validation_error(
                f"Cannot close check with status '{check.status.value}'. Check must be paid before closing."
            )
        check.status = CheckStatus.CLOSED
        check.closed_at = datetime.now(timezone.utc)
        check.updated_at = datetime.now(timezone.utc)
    else:
        raise_validation_error(
            f"Cannot close check with status '{check.status.value}'. Check must be paid before closing."
        )
    
    db.commit()
    db.refresh(check)
    return check


def mark_check_paid_by_payment_service(db: Session, check_id: int, payment_id: int):
    """
    Called by payment-service after successful payment completion
    This is the callback endpoint for inter-service communication
    """
    check = db.query(Check).filter(Check.id == check_id).first()
    if not check:
        raise_not_found("Check", check_id)
    
    # Update check status to paid
    check.status = CheckStatus.PAID
    check.paid_at = datetime.now(timezone.utc)
    
    db.commit()
    db.refresh(check)
    
    logger.info(f"Check {check_id} marked as paid by payment-service (payment_id: {payment_id})")
    return check


