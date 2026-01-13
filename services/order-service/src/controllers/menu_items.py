from sqlalchemy.orm import Session
from fastapi import HTTPException, status, Response, Depends
from shared.models import menu_items as model
from sqlalchemy.exc import SQLAlchemyError
from shared.utils.error_handlers import handle_database_error


def create(db: Session, request):
    new_item = model.MenuItem(
        name=request.name,
        description=request.description,
        price=request.price,
        calories=request.calories,
        food_category=request.food_category,
    )

    try:
        db.add(new_item)
        db.commit()
        db.refresh(new_item)
    except SQLAlchemyError as e:
        raise handle_database_error(e, operation="menu item creation", log_context={"name": request.name})

    return new_item

def read_all(db: Session):
    try:
        result = db.query(model.MenuItem).all()
    except SQLAlchemyError as e:
        raise handle_database_error(e, operation="menu items read_all")
    return result


def read_one(db: Session, item_id):
    try:
        item = db.query(model.MenuItem).filter(model.MenuItem.id == item_id).first()
        if not item:
            raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail="Id not found!")
    except SQLAlchemyError as e:
        raise handle_database_error(e, operation="menu item read_one", log_context={"item_id": item_id})
    return item


def update(db: Session, item_id, request):
    try:
        item = db.query(model.MenuItem).filter(model.MenuItem.id == item_id)
        if not item.first():
            raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail="Id not found!")
        update_data = request.dict(exclude_unset=True)
        item.update(update_data, synchronize_session=False)
        db.commit()
    except SQLAlchemyError as e:
        raise handle_database_error(e, operation="menu item update", log_context={"item_id": item_id})
    return item.first()


def delete(db: Session, item_id):
    try:
        item = db.query(model.MenuItem).filter(model.MenuItem.id == item_id)
        if not item.first():
            raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail="Id not found!")
        item.delete(synchronize_session=False)
        db.commit()
    except SQLAlchemyError as e:
        raise handle_database_error(e, operation="menu item delete", log_context={"item_id": item_id})
    return Response(status_code=status.HTTP_204_NO_CONTENT)
