from sqlalchemy.orm import Session
from fastapi import HTTPException, status
from shared.repositories import BaseRepository
from ..models.payment_method import Payment
from ..models.checks import Check
from ..schemas.payment_method import PaymentCreate, PaymentUpdate, PaymentType, PaymentStatus
from decimal import Decimal

# Initialize repository
payment_repo = BaseRepository[Payment, PaymentCreate, PaymentUpdate](Payment)


def create(db: Session, request: PaymentCreate):
    """Create payment"""
    # Validate check exists
    check = db.query(Check).filter(Check.id == request.check_id).first()
    if not check:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Check with id {request.check_id} not found"
        )
    
    # validate payment amount
    if request.amount <= 0:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Payment amount must be greater than 0"
        )
    
    # check if total payments would exceed check total
    existing_payments = payment_repo.filter_by(
        db,
        check_id=request.check_id
    )
    existing_payments = [p for p in existing_payments if p.status in [PaymentStatus.COMPLETED, PaymentStatus.PENDING]]
    
    total_existing = sum(payment.amount for payment in existing_payments)
    if total_existing + request.amount > check.total_amount:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Payment amount would exceed check total. Check total: {check.total_amount}, existing payments: {total_existing}, attempted payment: {request.amount}"
        )
    
    # Set payment status based on type
    payment_status = PaymentStatus.COMPLETED if request.payment_type == PaymentType.CASH else request.status
    
    # Create modified request with status
    payment_data = request.model_dump()
    payment_data['status'] = payment_status
    
    new_payment = Payment(**payment_data)
    db.add(new_payment)
    db.commit()
    db.refresh(new_payment)
    return new_payment


def read_all(db: Session):
    return payment_repo.get_all(db)


def read_one(db: Session, item_id: int):
    return payment_repo.get_or_404(db, item_id)


def update(db: Session, item_id: int, request: PaymentUpdate):
    return payment_repo.update(db, item_id, request)


def delete(db: Session, item_id: int):
    payment_repo.delete(db, item_id)


def read_by_check(db: Session, check_id: int):
    """Get all payments for a specific check"""
    # verify check exists
    check = db.query(Check).filter(Check.id == check_id).first()
    if not check:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Check with id {check_id} not found"
        )
    
    return payment_repo.filter_by(db, check_id=check_id)


def get_check_payment_summary(db: Session, check_id: int):
    """Get payment summary for a check including total paid and remaining balance"""
    # verify check exists
    check = db.query(Check).filter(Check.id == check_id).first()
    if not check:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Check with id {check_id} not found"
        )
    
    payments = payment_repo.filter_by(db, check_id=check_id)
    payments = [p for p in payments if p.status in [PaymentStatus.COMPLETED, PaymentStatus.PENDING]]
    
    total_paid = sum(payment.amount for payment in payments if payment.status == PaymentStatus.COMPLETED)
    total_pending = sum(payment.amount for payment in payments if payment.status == PaymentStatus.PENDING)
    remaining_balance = check.total_amount - total_paid - total_pending
    
    return {
        "check_id": check_id,
        "check_total": check.total_amount,
        "total_paid": total_paid,
        "total_pending": total_pending,
        "remaining_balance": remaining_balance,
        "is_fully_paid": remaining_balance <= 0,
        "payments": payments
    }


def create_split_payment(db: Session, check_id: int, split_amounts: list[dict]):
    """Create multiple payments for bill splitting
    
    Args:
        check_id: The check to split payment for
        split_amounts: List of dicts with payment details: 
                      [{"amount": 25.00, "payment_type": "cash"}, {"amount": 30.00, "payment_type": "credit_card", "card_number": "1234"}]
    """
    # verify check exists
    check = db.query(Check).filter(Check.id == check_id).first()
    if not check:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Check with id {check_id} not found"
        )
    
    # Validate total split amount matches check total
    total_split = sum(Decimal(str(split["amount"])) for split in split_amounts)
    if total_split != check.total_amount:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Split payment total ({total_split}) does not match check total ({check.total_amount})"
        )
    
    # check for existing payments
    existing_payments = payment_repo.filter_by(db, check_id=check_id)
    existing_payments = [p for p in existing_payments if p.status in [PaymentStatus.COMPLETED, PaymentStatus.PENDING]]
    
    if existing_payments:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Cannot create split payments - check already has existing payments"
        )
    
    created_payments = []
    for split in split_amounts:
        payment_type = PaymentType(split["payment_type"])
        payment_status = PaymentStatus.COMPLETED if payment_type == PaymentType.CASH else PaymentStatus.PENDING
        
        new_payment = Payment(
            check_id=check_id,
            amount=Decimal(str(split["amount"])),
            payment_type=payment_type,
            status=payment_status,
            card_number=split.get("card_number")
        )
        
        db.add(new_payment)
        created_payments.append(new_payment)
    
    db.commit()
    
    # refresh all payments
    for payment in created_payments:
        db.refresh(payment)
    
    return created_payments
