from sqlalchemy.orm import Session
from fastapi import HTTPException, status, Response, Depends
from ..models import resources as model
from sqlalchemy.exc import SQLAlchemyError
from shared.utils.error_handlers import handle_database_error


def create(db: Session, request):
    new_item = model.Resource(
        item=request.item,
        amount=request.amount
    )

    try:
        db.add(new_item)
        db.commit()
        db.refresh(new_item)
    except SQLAlchemyError as e:
        raise handle_database_error(e, operation="resource creation", log_context={"item": request.item})

    return new_item

def read_all(db: Session):
    try:
        result = db.query(model.Resource).all()
    except SQLAlchemyError as e:
        raise handle_database_error(e, operation="resources read_all")
    return result


def read_one(db: Session, item_id):
    try:
        item = db.query(model.Resource).filter(model.Resource.id == item_id).first()
        if not item:
            raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail="Id not found!")
    except SQLAlchemyError as e:
        raise handle_database_error(e, operation="resource read_one", log_context={"item_id": item_id})
    return item


def update(db: Session, item_id, request):
    try:
        item = db.query(model.Resource).filter(model.Resource.id == item_id)
        if not item.first():
            raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail="Id not found!")
        update_data = request.dict(exclude_unset=True)
        item.update(update_data, synchronize_session=False)
        db.commit()
    except SQLAlchemyError as e:
        raise handle_database_error(e, operation="resource update", log_context={"item_id": item_id})
    return item.first()


def delete(db: Session, item_id):
    try:
        item = db.query(model.Resource).filter(model.Resource.id == item_id)
        if not item.first():
            raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail="Id not found!")
        item.delete(synchronize_session=False)
        db.commit()
    except SQLAlchemyError as e:
        raise handle_database_error(e, operation="resource delete", log_context={"item_id": item_id})
    return Response(status_code=status.HTTP_204_NO_CONTENT)