from fastapi import APIRouter, Depends
from sqlalchemy.orm import Session
from typing import Optional
from ..services import customer_services
from ..schemas import menu_items as schema
from ..schemas import order_details as order_detail_schema
from ..schemas import payment_method as payment_schema
from ..controllers import payment_method as payment_controller
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


@router.post("/add-to-cart", response_model=order_detail_schema.OrderDetail)
def add_to_cart(menu_item_id: int, quantity: int, 
                customer_id: Optional[int] = None, order_id: Optional[int] = None, 
                db: Session = Depends(get_db)):
    """
    Add a menu item to the cart (order)
    If no order_id is provided, creates a new order
    """
    return customer_services.add_to_cart(
        db=db, 
        menu_item_id=menu_item_id, 
        quantity=quantity, 
        order_id=order_id
    )

# add customer info
# email etc isn't required if in restaurant


@router.post("/add-payment", response_model=payment_schema.Payment)
def add_payment_method(order_id: int, payment_type: payment_schema.PaymentType,
                       db: Session = Depends(get_db)):
    """
    Add payment to an order
    """
    payment_request = payment_schema.PaymentCreate(
        order_id=order_id,
        payment_type=payment_type,
        status=payment_schema.PaymentStatus.COMPLETED
    )
    return payment_controller.create(db=db, request=payment_request)

# checkout
# payment needed
# customer info needed