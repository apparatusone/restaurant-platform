from fastapi import APIRouter, Depends, Response, Cookie, HTTPException
from sqlalchemy.orm import Session
from typing import Optional
from ..services import customer_services
from ..schemas import menu_items as schema
from ..schemas import order_details as order_detail_schema
from ..schemas import payment_method as payment_schema
from ..schemas import customers as customer_schema
from ..schemas import orders as order_schema
from ..controllers import payment_method as payment_controller
from ..controllers import customers as customer_controller
from ..controllers import orders as order_controller
from ..dependencies.database import get_db
from ..schemas.orders import OrderType

router = APIRouter(
    tags=['Customer Actions'],
    prefix="/customer"
)


@router.get("/menu", response_model=list[schema.MenuItems])
def get_available_menu(filter_category: Optional[customer_services.FilterCategory] = None, db: Session = Depends(get_db)):
    """
    Get available menu items based on availability
    """
    return customer_services.get_menu(db=db, filter_category=filter_category)


@router.get("/menu/{item_name}", response_model=schema.MenuItems)
def get_menu_item_by_name(item_name: str, db: Session = Depends(get_db)):
    """
    Search for a particular menu item by name
    """
    return customer_services.get_menu_item_by_name(db=db, item_name=item_name)


@router.post("/add-to-cart")
def add_to_cart(response: Response, menu_item_id: int, quantity: int, 
                customer_id: Optional[int] = None, 
                order_id: Optional[int] = Cookie(None),
                db: Session = Depends(get_db)):
    """
    Add a menu item to the cart (order)
    If no order_id is provided, creates a new order
    """
    result = customer_services.add_to_cart(
        db=db, 
        menu_item_id=menu_item_id, 
        quantity=quantity, 
        order_id=order_id
    )
    
    # Set cookie with order_id in the customers browser
    # this would probably be implemented with a json token with a lot more complexity
    response.set_cookie(key="order_id", value=str(result["order_id"]), max_age=3600)
    return result


@router.delete("/remove-from-cart")
def remove_from_cart(menu_item_id: int, order_id: Optional[int] = Cookie(None, include_in_schema=False), db: Session = Depends(get_db)):
    """
    Removes an item from the customers cart
    """
    if not order_id:
        raise HTTPException(status_code=400, detail="No active order found")
        
    return customer_services.remove_item_from_cart(
        db=db, 
        order_id=order_id,
        menu_item_id=menu_item_id
    )


@router.post("/choose-order-type")
def choose_order_type(type: OrderType, order_id: Optional[int] = Cookie(None, include_in_schema=False), db: Session = Depends(get_db)):
    """
    Specify if order is takeout, etc
    """
    if not order_id:
        raise HTTPException(status_code=400, detail="No active order found")
        
    return customer_services.choose_order_type(
        type=type,
        order_id=order_id,
        db=db
    )
    

@router.post("/add-customer-information", response_model=order_schema.Order)
def add_customer_information(customer_name: str, 
                           customer_email: Optional[str] = None,
                           customer_phone: Optional[int] = None,
                           customer_address: Optional[str] = None,
                           order_id: Optional[int] = Cookie(None, include_in_schema=False),
                           db: Session = Depends(get_db)):
    """
    Add customer information to an order
    """
    if not order_id:
        raise HTTPException(status_code=400, detail="No active order found")
        
    return customer_services.add_customer_information(
        db=db,
        order_id=order_id,
        customer_name=customer_name,
        customer_email=customer_email,
        customer_phone=customer_phone,
        customer_address=customer_address
    )


@router.post("/add-payment", response_model=payment_schema.Payment)
def add_payment_method(payment_type: payment_schema.PaymentType,
                       card_number: Optional[str] = None, 
                       order_id: Optional[int] = Cookie(None, include_in_schema=False),
                       db: Session = Depends(get_db)):
    """
    Add payment to an order
    card_number is optional and only used for card payments
    """
    if not order_id:
        raise HTTPException(status_code=400, detail="No active order found")
        
    return customer_services.add_payment_method(
        db=db,
        order_id=order_id,
        payment_type=payment_type,
        card_number=card_number
    )


@router.post("/add-promo-code-to-order", response_model=order_schema.Order)
def add_promo_code_to_order(promo_code: str, 
                           order_id: Optional[int] = Cookie(None, include_in_schema=False),
                           db: Session = Depends(get_db)):
    """
    Add a promo code to an order
    """
    if not order_id:
        raise HTTPException(status_code=400, detail="No active order found")
        
    return customer_services.add_promo_code(
        db=db,
        order_id=order_id,
        promo_code=promo_code
    )


@router.post("/checkout")
def checkout(response: Response, 
             order_id: Optional[int] = Cookie(None, include_in_schema=False),
             db: Session = Depends(get_db)):
    """
    Send the order to the restaurant.
    """
    if not order_id:
        raise HTTPException(status_code=400, detail="No active order found")
        
    result = customer_services.checkout(
        db=db,
        order_id=order_id,
        response=response
    )
    
    return result