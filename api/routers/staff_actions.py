from fastapi import APIRouter, Depends, Path, Body, Query
from sqlalchemy.orm import Session
from datetime import date
from ..services import staff as staff_services
from ..schemas import menu_items as schema
from ..schemas import promotions as promotion_schema
from ..dependencies.database import engine, get_db
from ..models.orders import StatusType

# holds common actions made by the staff

router = APIRouter(
    tags=['Staff Actions'],
    prefix="/staff_actions",
)

@router.get("/view-ingredients")
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


@router.get("/revenue")
def get_daily_revenue(
        target_date: date = Query(
            default=date.today(),
            description="Defaults to today's date"
        ), 
        db: Session = Depends(get_db)
):
    """
    Get total revenue and completed orders for a specific date (YYYY-MM-DD format)
    """
    return staff_services.get_daily_revenue(db=db, target_date=target_date)


@router.get("/reviews/{rating}")
def review_feedback(
    rating: int = Path(..., ge=1, le=5, description="Rating value between 1 and 5"),
    db: Session = Depends(get_db)
):
    """
    Get a list of menu items based on their review rating, list reviews
    """
    return staff_services.review_feedback(db=db, rating=rating)


@router.get("/orders/status/{status}")
def get_orders_by_status(status: StatusType, db: Session = Depends(get_db)):
    """
    Get orders filtered by status
    """
    return staff_services.get_orders_by_status(db=db, status=status)


@router.put("/orders/{order_id}/status")
def update_order_status(order_id: int, status: StatusType, db: Session = Depends(get_db)):
    """
    Update the status of an order
    """
    return staff_services.update_order_status(db=db, order_id=order_id, status=status)


@router.get("/orders/date-range")
def get_orders_by_date_range(start_date: date, end_date: date, db: Session = Depends(get_db)):
    """
    Get orders within a date range (YYYY-MM-DD)
    """
    return staff_services.get_orders_by_date_range(db=db, start_date=start_date, end_date=end_date)


@router.post("/add-menu-item")
def add_menu_item(
    name: str,
    price: float,
    food_category: schema.FoodCategory,
    calories: int,
    description: str = None,
    resources: list[schema.ResourceRequirement] = Body(...),
    db: Session = Depends(get_db)
):
    """
    Create a new menu item with necessary resources
    """
    request = schema.MenuItemsCreateWithResources(
        name=name,
        price=price,
        food_category=food_category,
        calories=calories,
        description=description,
        resources=resources
    )
    
    return staff_services.add_menu_item(db=db, request=request)


@router.put("/update-stock")
def update_stock(
    resource_name: str,
    amount_change: int,
    db: Session = Depends(get_db)
):
    """
    Add or remove stock
    
    Use positive numbers to add stock, negative numbers to remove stock
    """
    return staff_services.update_stock(db=db, resource_name=resource_name, amount_change=amount_change)