from fastapi import APIRouter, Depends
from sqlalchemy.orm import Session
from ..services import customer_services
from ..schemas import menu_items as schema
from ..dependencies.database import get_db

router = APIRouter(
    tags=['Customer Actions'],
    prefix="/customer"
)


@router.get("/menu", response_model=list[schema.MenuItems])
def get_available_menu(db: Session = Depends(get_db)):
    """
    Get available menu items based on resource availability
    """
    return customer_services.get_menu(db=db)


@router.get("/menu/{item_name}", response_model=schema.MenuItems)
def get_menu_item_by_name(item_name: str, db: Session = Depends(get_db)):
    """
    Search for a particular menu item by name
    """
    return customer_services.get_menu_item_by_name(db=db, item_name=item_name)