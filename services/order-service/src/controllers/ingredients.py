from sqlalchemy.orm import Session
from sqlalchemy import and_
from shared.repositories import BaseRepository
from ..models.ingredients import Ingredient
from ..schemas.ingredients import IngredientCreate, IngredientUpdate

# Initialize repository
ingredient_repo = BaseRepository[Ingredient, IngredientCreate, IngredientUpdate](Ingredient)


def create(db: Session, request: IngredientCreate):
    return ingredient_repo.create(db, request)


def read_all(db: Session, restaurant_id: int = None):
    """Get all ingredients, optionally filtered by restaurant_id"""
    if restaurant_id is not None:
        return db.query(Ingredient).filter(Ingredient.restaurant_id == restaurant_id).all()
    return ingredient_repo.get_all(db)


def read_one(db: Session, item_id: int, restaurant_id: int = None):
    """Get a single ingredient, optionally filtered by restaurant_id"""
    if restaurant_id is not None:
        ingredient = db.query(Ingredient).filter(
            and_(Ingredient.id == item_id, Ingredient.restaurant_id == restaurant_id)
        ).first()
        if not ingredient:
            from fastapi import HTTPException
            raise HTTPException(status_code=404, detail="Ingredient not found")
        return ingredient
    return ingredient_repo.get_or_404(db, item_id)


def update(db: Session, item_id: int, request: IngredientUpdate, restaurant_id: int = None):
    """Update an ingredient, optionally filtered by restaurant_id"""
    if restaurant_id is not None:
        ingredient = db.query(Ingredient).filter(
            and_(Ingredient.id == item_id, Ingredient.restaurant_id == restaurant_id)
        ).first()
        if not ingredient:
            from fastapi import HTTPException
            raise HTTPException(status_code=404, detail="Ingredient not found")
        
        # Update fields
        update_data = request.model_dump(exclude_unset=True)
        for field, value in update_data.items():
            setattr(ingredient, field, value)
        
        db.commit()
        db.refresh(ingredient)
        return ingredient
    
    return ingredient_repo.update(db, item_id, request)


def delete(db: Session, item_id: int, restaurant_id: int = None):
    """Delete an ingredient, optionally filtered by restaurant_id"""
    if restaurant_id is not None:
        ingredient = db.query(Ingredient).filter(
            and_(Ingredient.id == item_id, Ingredient.restaurant_id == restaurant_id)
        ).first()
        if not ingredient:
            from fastapi import HTTPException
            raise HTTPException(status_code=404, detail="Ingredient not found")
        
        db.delete(ingredient)
        db.commit()
        return
    
    ingredient_repo.delete(db, item_id)
