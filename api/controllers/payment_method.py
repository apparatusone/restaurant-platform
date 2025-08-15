from sqlalchemy.orm import Session
from fastapi import HTTPException, status, Response, Depends
from ..models import payment_method as model
from ..models.checks import Check
from ..models.orders import Order
from sqlalchemy.exc import SQLAlchemyError
from ..schemas.payment_method import PaymentType, PaymentStatus
from decimal import Decimal


def create(db: Session, request):
    check = db.query(Check).filter(Check.id == request.check_id).first()
    if not check:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Check with id {request.check_id} not found"
        )
    
    # validate order_id if provided
    if request.order_id:
        order = db.query(Order).filter(Order.id == request.order_id).first()
        if not order:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"Order with id {request.order_id} not found"
            )
        
        # ensure order belongs to the check
        if order.check_id != request.check_id:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail=f"Order {request.order_id} does not belong to check {request.check_id}"
            )
    
    # validate payment amount
    if request.amount <= 0:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Payment amount must be greater than 0"
        )
    
    # check if total payments would exceed check total
    existing_payments = db.query(model.Payment).filter(
        model.Payment.check_id == request.check_id,
        model.Payment.status.in_([PaymentStatus.COMPLETED, PaymentStatus.PENDING])
    ).all()
    
    total_existing = sum(payment.amount for payment in existing_payments)
    if total_existing + request.amount > check.total_amount:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Payment amount would exceed check total. Check total: {check.total_amount}, existing payments: {total_existing}, attempted payment: {request.amount}"
        )

    payment_status = PaymentStatus.COMPLETED if request.payment_type == PaymentType.CASH else request.status

    new_item = model.Payment(
        check_id=request.check_id,
        order_id=request.order_id,
        amount=request.amount,
        payment_type=request.payment_type,
        status=payment_status,
        card_number=request.card_number
    )

    try:
        db.add(new_item)
        db.commit()
        db.refresh(new_item)
    except SQLAlchemyError as e:
        error = str(e.__dict__['orig'])
        raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail=error)

    return new_item

def read_all(db: Session):
    try:
        result = db.query(model.Payment).all()
    except SQLAlchemyError as e:
        error = str(e.__dict__['orig'])
        raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail=error)
    return result


def read_one(db: Session, item_id):
    try:
        item = db.query(model.Payment).filter(model.Payment.id == item_id).first()
        if not item:
            raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail="Id not found!")
    except SQLAlchemyError as e:
        error = str(e.__dict__['orig'])
        raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail=error)
    return item


def update(db: Session, item_id, request):
    try:
        item = db.query(model.Payment).filter(model.Payment.id == item_id)
        if not item.first():
            raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail="Id not found!")
        update_data = request.dict(exclude_unset=True)
        item.update(update_data, synchronize_session=False)
        db.commit()
    except SQLAlchemyError as e:
        error = str(e.__dict__['orig'])
        raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail=error)
    return item.first()


def delete(db: Session, item_id):
    try:
        item = db.query(model.Payment).filter(model.Payment.id == item_id)
        if not item.first():
            raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail="Id not found!")
        item.delete(synchronize_session=False)
        db.commit()
    except SQLAlchemyError as e:
        error = str(e.__dict__['orig'])
        raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail=error)
    return Response(status_code=status.HTTP_204_NO_CONTENT)
