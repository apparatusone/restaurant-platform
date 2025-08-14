import random
import string
from datetime import datetime
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
from .print import print_receipt
from ..models.orders import Order

from decimal import Decimal
from enum import Enum

class FilterCategory(str, Enum):
    VEGETARIAN = "vegetarian"
    VEGAN = "vegan"
    GLUTEN_FREE = "gluten_free"
    REGULAR = "regular"


def can_order_be_made(db: Session, order_id: int):
    """
    check if an order can be made based on current resource availability.
    returns true/false
    returns insufficient resources
    """
    # Get all order details for this order
    order_details = db.query(OrderDetail).filter(OrderDetail.order_id == order_id).all()
    
    if not order_details:
        return True, "Order is empty"
    
    total_ingredient_needs = {}
    
    for order_detail in order_details:
        # get required ingredients
        ingredients = db.query(MenuItemIngredient).filter(
            MenuItemIngredient.menu_item_id == order_detail.menu_item_id
        ).all()
        
        for ingredient in ingredients:
            resource_id = ingredient.resource_id
            needed_amount = ingredient.amount * order_detail.amount
            
            if resource_id in total_ingredient_needs:
                total_ingredient_needs[resource_id] += needed_amount
            else:
                total_ingredient_needs[resource_id] = needed_amount
    
    missing_ingredients = []
    
    for resource_id, needed_amount in total_ingredient_needs.items():
        resource = db.query(Resource).filter(Resource.id == resource_id).first()
        
        if not resource or resource.amount < needed_amount:
            available = resource.amount if resource else 0
            missing_ingredients.append({
                "resource_name": resource.item if resource else "Unknown",
                "needed": needed_amount,
                "available": available
            })
    
    if missing_ingredients:
        return False, missing_ingredients
    
    return True, "Order can be made"


def get_menu(db: Session, filter_category: Optional[FilterCategory] = None):
    query = db.query(MenuItem)
    if filter_category:
        if filter_category == FilterCategory.VEGETARIAN:
            # vegetarian includes both vegetarian and vegan items
            query = query.filter(MenuItem.food_category.in_(['vegetarian', 'vegan']))
        else:
            query = query.filter(MenuItem.food_category == filter_category.value)
    
    all_menu_items = query.all()
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


def add_payment_method(db: Session, order_id: int, payment_type: payment_schema.PaymentType,
                       card_number: Optional[str] = None):
    payment_request = payment_schema.PaymentCreate(
        order_id=order_id,
        payment_type=payment_type,
        status=payment_schema.PaymentStatus.PENDING,
        card_number=card_number
    )

    if payment_type != payment_schema.PaymentType.CASH and card_number is None:
        raise HTTPException(status_code=400, detail=f"Card number required for {payment_type} payments.")

    # if customer enters a card number with cash, remove it
    if payment_type == payment_schema.PaymentType.CASH:
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
    
    return {"message": f"Promo code '{promo_code}' applied successfully"}


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


def process_payment(db: Session, order: Order, total_amount: float):
    from ..schemas.payment_method import PaymentType
    from .payment import process_stripe_payment
    
    if not order.payment:
        raise HTTPException(status_code=400, detail="no payment method found")

    # cash payments don't need processing
    if order.payment.payment_type == PaymentType.CASH:
        return True
    
    # convert to cents for stripe
    amount_cents = int(total_amount * 100)
    
    # get customer name for stripe metadata
    customer_name = order.customer.customer_name if order.customer else None
    
    # process with stripe
    payment_response = process_stripe_payment(amount_cents, order.id, customer_name)
    
    if not payment_response["success"]:
        error_msg = payment_response.get("error", f"payment failed with status: {payment_response.get('status', 'unknown')}")
        raise HTTPException(status_code=400, detail=error_msg)
    
    return payment_response


def update_raw_ingredients(db: Session, order_id: int):
    """
    Deduct raw ingredients based on order items
    """
    from ..models.orders import Order
    from ..models.order_details import OrderDetail
    from ..models.menu_item_ingredients import MenuItemIngredient
    from ..models.resources import Resource
    from fastapi import HTTPException
    
    order_details = db.query(OrderDetail).filter(OrderDetail.order_id == order_id).all()
    
    if not order_details:
        raise HTTPException(status_code=404, detail="No items found in order")
    
    for detail in order_details:
        menu_item_id = detail.menu_item_id
        quantity_ordered = detail.amount
        
        required_ingredients = db.query(MenuItemIngredient).filter(
            MenuItemIngredient.menu_item_id == menu_item_id
        ).all()
        
        # update each ingredient in  stock
        for ingredient in required_ingredients:
            resource = db.query(Resource).filter(Resource.id == ingredient.resource_id).first()
            
            if resource:
                total_needed = ingredient.amount * quantity_ordered
                
                if resource.amount < total_needed:
                    raise HTTPException(
                        status_code=400, 
                        detail=f"Insufficient {resource.item} inventory. Need {total_needed}, have {resource.amount}"
                    )
                
                resource.amount -= total_needed
    
    db.commit()
    return True





# TODO: this handles far too much, simplify
def checkout(db: Session, order_id: int, response=None):
    """
    Process checkout for an order
    """
    from ..models.promotions import Promotion
    from ..models.orders import Order
    from datetime import datetime
    from fastapi import HTTPException
    from ..schemas.orders import OrderStatus
    from ..schemas.orders import OrderUpdate
    from ..config.restaurant import TAX_RATE

    # Get the order
    order = db.query(Order).filter(Order.id == order_id).first()
    if not order:
        raise HTTPException(status_code=404, detail="Order not found")

    # first check if the order can be made (has enough ingredients)
    # this is a second check if 2 customers are ordering at the same time
    can_be_made, details = can_order_be_made(db, order_id)
    if not can_be_made:
        missing_items = ", ".join([f"{item['resource_name']} (need {item['shortage']} more)" for item in details])
        raise HTTPException(
            status_code=400, 
            detail=f"Order cannot be completed due to insufficient ingredients: {missing_items}"
        )

    # check if the order has payment
    if not order.payment:
        raise HTTPException(status_code=400, detail="No payment method found. Please add a payment method before checkout.")

    subtotal = calculate_order_total(db, order_id)
    
    # apply any promotion
    if order.promo_id:
        promotion = db.query(Promotion).filter(Promotion.id == order.promo_id).first()
        if promotion:
            discount_amount = subtotal * (promotion.discount_percent / 100)
            subtotal = subtotal - discount_amount

    # Calculate tax and total
    tax = subtotal * TAX_RATE
    total = subtotal + tax
    
    # process the payment
    payment_result = process_payment(db, order, total)
    
    # change order to paid and store final total
    try:
        order_update = OrderUpdate(paid=True, final_total=total)
        updated_order = order_controller.update(db=db, request=order_update, item_id=order_id)
        
        # update payment status to completed
        from ..models.payment_method import PaymentStatus
        if order.payment:
            order.payment.status = PaymentStatus.COMPLETED
            db.commit()
            
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to update order payment status: {str(e)}")

    # change order status
    try:
        order_update = OrderUpdate(status=OrderStatus.IN_PROGRESS)
        updated_order = order_controller.update(db=db, request=order_update, item_id=order_id)
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to update order status to in progress: {str(e)}")

    # update resources
    if not update_raw_ingredients(db, order_id):
        raise HTTPException(status_code=500, detail="Failed to update stock")

    # the "order date" and time need to be updated at checkout
    try:
        order_update = OrderUpdate(order_date=datetime.now())
        order_controller.update(db=db, request=order_update, item_id=order_id)
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to update order date: {str(e)}")

    # clear the browser cookie after successful checkout
    if response:
        response.delete_cookie(key="order_id")
    
    # TODO: Send to kitchen (print ticket)

    # handle receipt based on order type
    try:
        if order.order_type.value in ["dine_in", "takeout"]:
            print_receipt(order_id, subtotal=subtotal, tax=tax, total=total)
        elif order.order_type.value == "delivery":
            # Send email receipt for delivery orders
            # TODO: Implement email receipt functionality
            print(f"Email receipt should be sent for delivery order {order_id}")
    except Exception as e:
        # Log the error but don't fail the checkout
        print(f"Warning: Failed to process receipt for order {order_id}: {str(e)}") 

    # if takeout or delivery provide tracking number
    tracking_number = None
    # Check order type using string values to avoid enum comparison issues
    if order.order_type.value in ["takeout", "delivery"]:
        # only generate if no existing number
        if not order.tracking_number:
            from ..controllers.orders import generate_tracking_number
            from ..models.orders import OrderType
            
            # Convert string to enum for tracking number generation
            order_type_enum = OrderType.TAKEOUT if order.order_type.value == "takeout" else OrderType.DELIVERY
            tracking_number = generate_tracking_number(order_type_enum)
            
            # check for duplicate tracking number
            while db.query(Order).filter(Order.tracking_number == tracking_number).first():
                tracking_number = generate_tracking_number(order_type_enum)
            
            # update the order with the tracking number
            try:
                order_update = OrderUpdate(tracking_number=tracking_number)
                updated_order = order_controller.update(db=db, order_id=order_id, request=order_update)
                db.refresh(order)
            except Exception as e:
                raise HTTPException(status_code=500, detail=f"Failed to update tracking number: {str(e)}")
        else:
            tracking_number = order.tracking_number
    
    response_data = {
        "message": "Order successfully created",
        "total": round(total, 2)
    }
    
    if tracking_number:
        response_data["tracking_number"] = tracking_number
    
    return response_data


def get_tracking_information(db: Session, tracking_number: str):
    """
    Get order tracking information by tracking number
    """
    from ..models.orders import Order
    from fastapi import HTTPException
    
    # Find the order by tracking number
    order = db.query(Order).filter(Order.tracking_number == tracking_number).first()
    
    if not order:
        raise HTTPException(status_code=404, detail="Tracking number not found")
    
    return {
        "tracking_number": tracking_number,
        "status": order.status.value
    }


def choose_order_type(db: Session, order_id: int, type):
    """
    Update the order type (dine_in, takeout, delivery)
    Tracking number will be generated at checkout for takeout and delivery orders
    """
    from ..schemas.orders import OrderUpdate
    from ..controllers import orders as order_controller
    
    # create the order update with the new type
    order_update = OrderUpdate(order_type=type)
    
    return order_controller.update(db=db, request=order_update, item_id=order_id)


def get_reviews(db: Session):
    """
    Get menu items that have reviews and the reviews
    """
    from ..models.menu_items import MenuItem
    from ..models.reviews import Reviews
    
    # get menu items that have reviews
    menu_items_with_reviews = db.query(MenuItem).join(Reviews).distinct().all()
    
    result = []
    for menu_item in menu_items_with_reviews:
        # get reviews
        reviews = db.query(Reviews).filter(Reviews.menu_item_id == menu_item.id).all()
        
        menu_item_data = {
            "menu_item_name": menu_item.name,
            "reviews": [
                {
                    "customer_name": review.customer_name,
                    "rating": review.rating,
                    "review_text": review.review_text,
                    "created_at": review.created_at
                }
                for review in reviews
            ]
        }
        result.append(menu_item_data)
    
    return result