from sqlalchemy.orm import Session
from fastapi import HTTPException, status
from shared.repositories import BaseRepository
from shared.utils.http_client import ResilientHttpClient
from ..models.payment import Payment, PaymentType, PaymentStatus
from ..schemas.payment import PaymentCreate, PaymentUpdate
from decimal import Decimal
import os
import logging

logger = logging.getLogger(__name__)

# Initialize repository
payment_repo = BaseRepository[Payment, PaymentCreate, PaymentUpdate](Payment)

# Initialize HTTP client for order-service
ORDER_SERVICE_URL = os.getenv("ORDER_SERVICE_URL", "http://localhost:8002")
order_service_client = ResilientHttpClient(base_url=ORDER_SERVICE_URL)


async def notify_order_service(check_id: int, payment_id: int) -> bool:
    """
    Notify order-service that a payment has been completed
    
    Args:
        check_id: The check ID that was paid
        payment_id: The payment ID
    
    Returns:
        True if notification was successful, False otherwise
    """
    try:
        logger.info(f"Notifying order-service of payment completion for check {check_id}")
        
        # Call order-service to update check status to PAID
        response = await order_service_client.put(
            f"/checks/{check_id}/mark-paid?payment_id={payment_id}"
        )
        
        if response.status_code == 200:
            logger.info(f"Successfully notified order-service of payment {payment_id} for check {check_id}")
            return True
        else:
            logger.error(
                f"Failed to notify order-service: status {response.status_code}, "
                f"response: {response.text}"
            )
            return False
    
    except Exception as e:
        logger.error(f"Error notifying order-service for check {check_id}: {e}")
        return False


async def create(db: Session, request: PaymentCreate):
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
    
    # Create payment with correct status
    payment_data = request.model_dump()
    payment_data['status'] = payment_status
    
    new_payment = Payment(**payment_data)
    db.add(new_payment)
    db.commit()
    db.refresh(new_payment)
    
    # Notify order-service if payment is completed
    if new_payment.status == PaymentStatus.COMPLETED:
        notification_success = await notify_order_service(
            check_id=new_payment.check_id,
            payment_id=new_payment.id
        )
        
        if not notification_success:
            logger.warning(
                f"Payment {new_payment.id} created but order-service notification failed. "
                f"Check {new_payment.check_id} may need manual status update."
            )
    
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
