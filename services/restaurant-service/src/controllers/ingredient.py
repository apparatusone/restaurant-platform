from sqlalchemy.orm import Session
from fastapi import HTTPException, status
from shared.repositories import BaseRepository
from ..models.ingredient import Ingredient
from ..schemas.ingredient import IngredientCreate, IngredientUpdate

# Initialize repository
ingredient_repo = BaseRepository[Ingredient, IngredientCreate, IngredientUpdate](Ingredient)


def create(db: Session, request: IngredientCreate):
    return ingredient_repo.create(db, request)


def read_all(db: Session, restaurant_id: int = None):
    """Get all ingredients, optionally filtered by restaurant_id"""
    if restaurant_id is not None:
        return ingredient_repo.filter_by(db, restaurant_id=restaurant_id)
    return ingredient_repo.get_all(db)


def read_one(db: Session, item_id: int, restaurant_id: int = None):
    """Get a single ingredient, optionally filtered by restaurant_id"""
    ingredient = ingredient_repo.get_or_404(db, item_id)
    
    # Verify restaurant_id if provided (multi-tenant check)
    if restaurant_id is not None and ingredient.restaurant_id != restaurant_id:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Ingredient not found"
        )
    
    return ingredient


def update(db: Session, item_id: int, request: IngredientUpdate, restaurant_id: int = None):
    """Update an ingredient, optionally filtered by restaurant_id"""
    ingredient = ingredient_repo.get_or_404(db, item_id)
    
    # Verify restaurant_id if provided (multi-tenant check)
    if restaurant_id is not None and ingredient.restaurant_id != restaurant_id:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Ingredient not found"
        )
    
    return ingredient_repo.update(db, item_id, request)


def delete(db: Session, item_id: int, restaurant_id: int = None):
    """Delete an ingredient, optionally filtered by restaurant_id"""
    ingredient = ingredient_repo.get_or_404(db, item_id)
    
    # Verify restaurant_id if provided (multi-tenant check)
    if restaurant_id is not None and ingredient.restaurant_id != restaurant_id:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Ingredient not found"
        )
    
    ingredient_repo.delete(db, item_id)
