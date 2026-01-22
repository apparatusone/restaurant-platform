from sqlalchemy.orm import Session
from fastapi import HTTPException, status
from shared.repositories import BaseRepository
from ..models.recipes import Recipe
from ..schemas.recipes import RecipeCreate, RecipeUpdate

# Initialize repository
recipe_repo = BaseRepository[Recipe, RecipeCreate, RecipeUpdate](Recipe)


def create(db: Session, request: RecipeCreate):
    """Create recipe"""
    # require recipe amount to be greater than 0
    if request.amount <= 0:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Amount must be greater than 0"
        )
    
    return recipe_repo.create(db, request)


def read_all(db: Session):
    return recipe_repo.get_all(db)


def read_one(db: Session, item_id: int):
    return recipe_repo.get_or_404(db, item_id)


def update(db: Session, item_id: int, request: RecipeUpdate):
    """Update recipe"""
    # require recipe amount to be greater than 0
    update_data = request.model_dump(exclude_unset=True)
    if 'amount' in update_data and update_data['amount'] <= 0:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Amount must be greater than 0"
        )
    
    return recipe_repo.update(db, item_id, request)


def delete(db: Session, item_id: int):
    recipe_repo.delete(db, item_id)
