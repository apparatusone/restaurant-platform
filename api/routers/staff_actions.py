from fastapi import APIRouter, Depends, FastAPI, status, Response
from sqlalchemy.orm import Session
from datetime import date
from ..services import staff_services
from ..schemas import menu_items as schema
from ..schemas import promotions as promotion_schema
from ..dependencies.database import engine, get_db

# holds common actions made by the staff

router = APIRouter(
    tags=['Staff Actions'],
    prefix="/staff_actions",
)

@router.get("/")
def view_ingredients(menu_item_id: int, quantity: int, db: Session = Depends(get_db)):
    return staff_services.get_required_ingredients(db, menu_item_id, quantity)


# promotions staff actions
@router.get("/promotions/code/{promo_code}", response_model=promotion_schema.Promotion)
def get_promotion_by_code(promo_code: str, db: Session = Depends(get_db)):
    return staff_services.get_promotion_by_code(db, promo_code=promo_code)


@router.put("/promotions/code/{promo_code}", response_model=promotion_schema.Promotion)
def update_promotion_by_code(promo_code: str, request: promotion_schema.PromotionUpdate, db: Session = Depends(get_db)):
    return staff_services.update_promotion_by_code(db=db, promo_code=promo_code, request=request)


@router.delete("/promotions/code/{promo_code}")
def delete_promotion_by_code(promo_code: str, db: Session = Depends(get_db)):
    return staff_services.delete_promotion_by_code(db=db, promo_code=promo_code)


@router.get("/revenue/{target_date}")
def get_daily_revenue(target_date: date, db: Session = Depends(get_db)):
    """
    Get total revenue for a specific date (YYYY-MM-DD format)
    """
    return staff_services.get_daily_revenue(db=db, target_date=target_date)