from sqlalchemy.orm import Session
from fastapi import HTTPException
from typing import Optional
from ..models.menu_items import MenuItem
from ..models.menu_item_ingredients import MenuItemIngredient
from ..models.resources import Resource
from ..schemas import orders as order_schema
from ..schemas import order_details as order_detail_schema
from ..schemas import payment_method as payment_schema
from ..schemas import customers as customer_schema
from ..controllers import orders as order_controller
from ..controllers import order_details as order_detail_controller
from ..controllers import payment_method as payment_controller
from ..controllers import customers as customer_controller
from ..schemas.payment_method import PaymentType
from ..models.order_details import OrderDetail
from ..models.menu_items import MenuItem
from decimal import Decimal


def get_menu(db: Session):
    all_menu_items = db.query(MenuItem).all()
    available_items = []
    
    for item in all_menu_items:
        # Check if all ingredients for this menu item are available
        ingredients = db.query(MenuItemIngredient).filter(
            MenuItemIngredient.menu_item_id == item.id
        ).all()
        
        if not ingredients:
            continue
            
        is_available = True
        for ingredient in ingredients:
            resource = db.query(Resource).filter(
                Resource.id == ingredient.resource_id
            ).first()
            
            if not resource or resource.amount < ingredient.amount:
                is_available = False
                break
        
        if is_available:
            available_items.append(item)
    
    return available_items


def get_menu_item_by_name(db: Session, item_name: str):
    menu_item = db.query(MenuItem).filter(
        MenuItem.name.ilike(f"%{item_name}%")
    ).first()
    
    if not menu_item:
        raise HTTPException(status_code=404, detail=f"Menu item '{item_name}' not found")
    
    # Check if the item is available (has sufficient ingredients)
    ingredients = db.query(MenuItemIngredient).filter(
        MenuItemIngredient.menu_item_id == menu_item.id
    ).all()
    
    # If no ingredients defined, item can't be made
    if not ingredients:
        raise HTTPException(status_code=400, detail=f"Menu item '{item_name}' cannot be prepared.")
    
    # Check ingredient availability
    for ingredient in ingredients:
        resource = db.query(Resource).filter(
            Resource.id == ingredient.resource_id
        ).first()
        
        if not resource or resource.amount < ingredient.amount:
            raise HTTPException(status_code=400, detail=f"Menu item '{item_name}' is currently unavailable.")
    
    return menu_item


def add_to_cart(db: Session, menu_item_id: int, quantity: int, customer_id: Optional[int] = None, order_id: Optional[int] = None):
    if order_id is None:
        order_request = order_schema.OrderCreate(
            description="Customer cart",
            status=order_schema.StatusType.PENDING,
            order_type=order_schema.OrderType.DINE_IN
        )
        new_order = order_controller.create(db=db, request=order_request)
        order_id = new_order.id
    
    # Add item to the order
    order_detail_request = order_detail_schema.OrderDetailCreate(
        order_id=order_id,
        menu_item_id=menu_item_id,
        amount=quantity
    )
    return order_detail_controller.create(db=db, request=order_detail_request)


def add_payment_method(db: Session, order_id: int, payment_type: payment_schema.PaymentType,
                       card_number: Optional[str] = None):
    payment_request = payment_schema.PaymentCreate(
        order_id=order_id,
        payment_type=payment_type,
        status=payment_schema.PaymentStatus.PENDING,
        card_number=card_number
    )

    if payment_type != PaymentType.CASH and card_number is None:
        raise HTTPException(status_code=400, detail=f"Card number required for {payment_type} payments.")

    # if customer enters a card number with cash, remove it
    if payment_type != PaymentType.CASH:
        payment_request.card_number = None

    return payment_controller.create(db=db, request=payment_request)


def add_customer_information(db: Session, order_id: int, customer_name: str,
                           customer_email: Optional[str] = None,
                           customer_phone: Optional[int] = None,
                           customer_address: Optional[str] = None):
    """
    Add customer information to an order
    """
    from ..models.customers import Customer
    
    existing_customer = None
    
    # If a customer exists with the provided email or phone number, don't create a new customer
    if customer_email:
        existing_customer = db.query(Customer).filter(Customer.customer_email == customer_email).first()
    
    if not existing_customer and customer_phone:
        existing_customer = db.query(Customer).filter(Customer.customer_phone == customer_phone).first()
    
    if existing_customer:
        # Update existing customer with new information
        customer_update = customer_schema.CustomerUpdate(
            customer_name=customer_name,
            customer_email=customer_email,
            customer_phone=customer_phone,
            customer_address=customer_address
        )
        customer = customer_controller.update(db=db, request=customer_update, item_id=existing_customer.id)
    else:
        # Create new customer
        customer_request = customer_schema.CustomerCreate(
            customer_name=customer_name,
            customer_email=customer_email,
            customer_phone=customer_phone,
            customer_address=customer_address
        )
        customer = customer_controller.create(db=db, request=customer_request)
    
    # Update the order with the customer_id
    order_update = order_schema.OrderUpdate(customer_id=customer.id)
    updated_order = order_controller.update(db=db, request=order_update, item_id=order_id)
    
    return updated_order


def calculate_order_total(db: Session, order_id: int) -> float:
    """
    Calculate the total amount for an order
    """
    # Get all order details on an order
    order_details = db.query(OrderDetail).join(MenuItem).filter(
        OrderDetail.order_id == order_id
    ).all()
    
    total = Decimal('0.00')
    
    for detail in order_details:
        # handle data types
        item_total = Decimal(detail.amount) * detail.menu_item.price
        total += item_total
    
    return float(round(total, 2))


def add_promo_code(db: Session, order_id: int, promo_code: str):
    """
    Apply a promo code to an order
    """
    from ..models.promotions import Promotion
    from ..models.orders import Order
    from ..schemas.orders import OrderUpdate
    from ..controllers import orders as order_controller
    from fastapi import HTTPException, status
    from datetime import datetime
    
    promo = db.query(Promotion).filter(Promotion.code == promo_code).first()
    
    if not promo:
        raise HTTPException(status_code=404, detail="Promo code not found.")
    
    # check if promo expired
    if promo.expiration_date and promo.expiration_date < datetime.now():
        raise HTTPException(status_code=400, detail="Promo code has expired")
    
    # check that the order exists
    order = db.query(Order).filter(Order.id == order_id).first()
    if not order:
        raise HTTPException(status_code=404, detail="Order not found.")
    
    order_update = OrderUpdate(promo_id=promo.id)
    updated_order = order_controller.update(db=db, request=order_update, item_id=order_id)
    
    return updated_order


def checkout(db: Session, order_id: int):
    """
    Process checkout for an order
    """
    from ..models.promotions import Promotion
    from ..models.orders import Order
    from datetime import datetime
    from fastapi import HTTPException

    # Get the order
    order = db.query(Order).filter(Order.id == order_id).first()
    if not order:
        raise HTTPException(status_code=404, detail="Order not found")

    total = calculate_order_total(db, order_id)
    print("original total: ", total)
    
    # Apply any promotion
    if order.promo_id:
        promotion = db.query(Promotion).filter(Promotion.id == order.promo_id).first()
        if promotion:
            discount_amount = total * (promotion.discount_percent / 100)
            total = total - discount_amount
            print(f"Applied {promotion.discount_percent}% discount: -{discount_amount}")

    # Apply tax
    TAX = 0.0475
    total = total * (1 + TAX)
    
    
    
    return round(total, 2)
    