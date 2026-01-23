from sqlalchemy.orm import Session
from fastapi import HTTPException, status
from shared.repositories import BaseRepository
from ..models.payment import Payment, PaymentType, PaymentStatus
from ..schemas.payment import PaymentCreate, PaymentUpdate
from decimal import Decimal

# Initialize repository
payment_repo = BaseRepository[Payment, PaymentCreate, PaymentUpdate](Payment)


def create(db: Session, request: PaymentCreate):
    """Create payment"""
    # Note: Check validation will need to be done via order-service API call
    # For now, we'll create the payment and assume check_id is valid
    # TODO: Add order-service client call to validate check exists
    
    # validate payment amount
    if request.amount <= 0:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Payment amount must be greater than 0"
        )
    
    # check if total payments would exceed check total
    # TODO: This validation will need to call order-service to get check total
    existing_payments = payment_repo.filter_by(
        db,
        check_id=request.check_id
    )
    existing_payments = [p for p in existing_payments if p.status in [PaymentStatus.COMPLETED, PaymentStatus.PENDING]]
    
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
    # Note: Check validation will need to be done via order-service API call
    # TODO: Add order-service client call to verify check exists
    
    return payment_repo.filter_by(db, check_id=check_id)


def get_check_payment_summary(db: Session, check_id: int):
    """Get payment summary for a check including total paid and remaining balance"""
    # Note: Check total will need to be fetched from order-service
    # TODO: Add order-service client call to get check details
    
    payments = payment_repo.filter_by(db, check_id=check_id)
    payments = [p for p in payments if p.status in [PaymentStatus.COMPLETED, PaymentStatus.PENDING]]
    
    total_paid = sum(payment.amount for payment in payments if payment.status == PaymentStatus.COMPLETED)
    total_pending = sum(payment.amount for payment in payments if payment.status == PaymentStatus.PENDING)
    
    # TODO: Fetch check_total from order-service
    # For now, return summary without check total and remaining balance
    return {
        "check_id": check_id,
        "total_paid": total_paid,
        "total_pending": total_pending,
        "payments": payments
    }
