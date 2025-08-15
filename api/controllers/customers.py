from sqlalchemy.orm import Session
from fastapi import HTTPException, status, Response, Depends
from ..models import customers as model
from ..utils.errors import (
    handle_sqlalchemy_error,
    raise_not_found
)
from sqlalchemy.exc import SQLAlchemyError


def create(db: Session, request):
    new_item = model.Customer(
        customer_name=request.customer_name,
        customer_email=request.customer_email,
        customer_phone=request.customer_phone,
        customer_address=request.customer_address,
    )

    try:
        db.add(new_item)
        db.commit()
        db.refresh(new_item)
    except SQLAlchemyError as e:
        handle_sqlalchemy_error(e).raise_exception()

    return new_item

def read_all(db: Session):
    try:
        result = db.query(model.Customer).all()
    except SQLAlchemyError as e:
        handle_sqlalchemy_error(e).raise_exception()
    return result


def read_one(db: Session, item_id):
    try:
        item = db.query(model.Customer).filter(model.Customer.id == item_id).first()
        if not item:
            raise_not_found("Customer", item_id)
    except SQLAlchemyError as e:
        handle_sqlalchemy_error(e).raise_exception()
    return item


def update(db: Session, item_id, request):
    try:
        item = db.query(model.Customer).filter(model.Customer.id == item_id)
        if not item.first():
            raise_not_found("Customer", item_id)
        update_data = request.dict(exclude_unset=True)
        item.update(update_data, synchronize_session=False)
        db.commit()
    except SQLAlchemyError as e:
        handle_sqlalchemy_error(e).raise_exception()
    return item.first()


def delete(db: Session, item_id):
    try:
        item = db.query(model.Customer).filter(model.Customer.id == item_id)
        if not item.first():
            raise_not_found("Customer", item_id)
        item.delete(synchronize_session=False)
        db.commit()
    except SQLAlchemyError as e:
        handle_sqlalchemy_error(e).raise_exception()
    return Response(status_code=status.HTTP_204_NO_CONTENT)
