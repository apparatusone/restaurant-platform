from fastapi import APIRouter, Depends
from sqlalchemy.orm import Session
from ..controllers import menu_item_ingredients as controller
from ..schemas import menu_item_ingredients as schema
from ..dependencies.database import engine, get_db

router = APIRouter(
    tags=['Menu Item Ingredients'],
    prefix="/menu_item_ingredients",
)


@router.post("/", response_model=schema.MenuItemIngredient)
def create(request: schema.MenuItemIngredientCreate, db: Session = Depends(get_db)):
    return controller.create(db=db, request=request)


@router.get("/", response_model=list[schema.MenuItemIngredient])
def read_all(db: Session = Depends(get_db)):
    return controller.read_all(db)


@router.get("/{item_id}", response_model=schema.MenuItemIngredient)
def read_one(item_id: int, db: Session = Depends(get_db)):
    return controller.read_one(db, item_id=item_id)


@router.put("/{item_id}", response_model=schema.MenuItemIngredient)
def update(item_id: int, request: schema.MenuItemIngredientUpdate, db: Session = Depends(get_db)):
    return controller.update(db=db, request=request, item_id=item_id)


@router.delete("/{item_id}")
def delete(item_id: int, db: Session = Depends(get_db)):
    return controller.delete(db=db, item_id=item_id)