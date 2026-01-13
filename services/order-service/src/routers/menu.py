from fastapi import APIRouter, Depends, Query
from sqlalchemy.orm import Session
from typing import Optional
from ..schemas import menu_items as schema
from ..services import customer as customer_services
from shared.dependencies.database import get_db

router = APIRouter(
    tags=['Menu'],
    prefix="/menu",
)


@router.get("/", response_model=list[schema.MenuItems])
def get_available_menu(
    category: Optional[customer_services.FilterCategory] = Query(None, description="Filter by food category"),
    available_only: Optional[bool] = Query(True, description="Show only available items"),
    db: Session = Depends(get_db)
):
    """
    Get menu items with optional filtering by category and availability
    """
    return customer_services.get_menu(db=db, filter_category=category)


@router.get("/{item_name}", response_model=schema.MenuItems)
def get_menu_item_by_name(item_name: str, db: Session = Depends(get_db)):
    """
    Search for a particular menu item by name
    """
    return customer_services.get_menu_item_by_name(db=db, item_name=item_name)