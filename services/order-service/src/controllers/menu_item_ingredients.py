from sqlalchemy.orm import Session
from fastapi import HTTPException, status, Response, Depends
from ..models import menu_item_ingredients as model
from sqlalchemy.exc import SQLAlchemyError
from shared.utils.error_handlers import handle_database_error


def create(db: Session, request):
    # require ingredient amount to be greater than 0
    if request.amount <= 0:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST, 
            detail="Amount must be greater than 0"
        )
    
    new_item = model.MenuItemIngredient(
        menu_item_id=request.menu_item_id,
        resource_id=request.resource_id,
        amount=request.amount
    )

    try:
        db.add(new_item)
        db.commit()
        db.refresh(new_item)
    except SQLAlchemyError as e:
        raise handle_database_error(
            e,
            operation="menu item ingredient creation",
            log_context={"menu_item_id": request.menu_item_id, "resource_id": request.resource_id}
        )

    return new_item

def read_all(db: Session):
    try:
        result = db.query(model.MenuItemIngredient).all()
    except SQLAlchemyError as e:
        raise handle_database_error(e, operation="menu item ingredients read_all")
    return result


def read_one(db: Session, item_id):
    try:
        item = db.query(model.MenuItemIngredient).filter(model.MenuItemIngredient.id == item_id).first()
        if not item:
            raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail="Id not found!")
    except SQLAlchemyError as e:
        raise handle_database_error(e, operation="menu item ingredient read_one", log_context={"item_id": item_id})
    return item


def update(db: Session, item_id, request):
    try:
        item = db.query(model.MenuItemIngredient).filter(model.MenuItemIngredient.id == item_id)
        if not item.first():
            raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail="Id not found!")
        
        update_data = request.dict(exclude_unset=True)
        
        # require ingredient amount to be greater than 0
        if 'amount' in update_data and update_data['amount'] <= 0:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST, 
                detail="Amount must be greater than 0"
            )
        
        item.update(update_data, synchronize_session=False)
        db.commit()
    except SQLAlchemyError as e:
        raise handle_database_error(e, operation="menu item ingredient update", log_context={"item_id": item_id})
    return item.first()


def delete(db: Session, item_id):
    try:
        item = db.query(model.MenuItemIngredient).filter(model.MenuItemIngredient.id == item_id)
        if not item.first():
            raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail="Id not found!")
        item.delete(synchronize_session=False)
        db.commit()
    except SQLAlchemyError as e:
        raise handle_database_error(e, operation="menu item ingredient delete", log_context={"item_id": item_id})
    return Response(status_code=status.HTTP_204_NO_CONTENT)