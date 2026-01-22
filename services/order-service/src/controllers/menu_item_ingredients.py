from sqlalchemy.orm import Session
from fastapi import HTTPException, status
from shared.repositories import BaseRepository
from ..models.menu_item_ingredients import MenuItemIngredient
from ..schemas.menu_item_ingredients import MenuItemIngredientCreate, MenuItemIngredientUpdate

# Initialize repository
ingredient_repo = BaseRepository[MenuItemIngredient, MenuItemIngredientCreate, MenuItemIngredientUpdate](MenuItemIngredient)


def create(db: Session, request: MenuItemIngredientCreate):
    """Create ingredient"""
    # require ingredient amount to be greater than 0
    if request.amount <= 0:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Amount must be greater than 0"
        )
    
    return ingredient_repo.create(db, request)


def read_all(db: Session):
    return ingredient_repo.get_all(db)


def read_one(db: Session, item_id: int):
    return ingredient_repo.get_or_404(db, item_id)


def update(db: Session, item_id: int, request: MenuItemIngredientUpdate):
    """Update ingredient"""
    # require ingredient amount to be greater than 0
    update_data = request.model_dump(exclude_unset=True)
    if 'amount' in update_data and update_data['amount'] <= 0:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Amount must be greater than 0"
        )
    
    return ingredient_repo.update(db, item_id, request)


def delete(db: Session, item_id: int):
    ingredient_repo.delete(db, item_id)