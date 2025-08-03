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
from ..models.orders import Order
from decimal import Decimal
from enum import Enum

class FilterCategory(str, Enum):
    VEGETARIAN = "vegetarian"
    VEGAN = "vegan"
    GLUTEN_FREE = "gluten_free"
    REGULAR = "regular"

def get_menu(db: Session, filter_category: Optional[FilterCategory] = None):
    # Apply filter
    query = db.query(MenuItem)
    if filter_category:
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


def add_to_cart(db: Session, menu_item_id: int, quantity: int, customer_id: Optional[int] = None, order_id: Optional[int] = None):
    from ..models.menu_items import MenuItem
    from ..models.menu_item_ingredients import MenuItemIngredient
    from ..models.resources import Resource
    from fastapi import HTTPException
    from ..models.order_details import OrderDetail
    
    # check if id (from cookie) exists or create an order
    from ..models.orders import Order
    
    if order_id is not None:
        # Check if the order from cookie still exists
        existing_order = db.query(Order).filter(Order.id == order_id).first()
        if not existing_order:
            # Order doesn't exist anymore, create a new one
            order_id = None
    
    if order_id is None:
        order_request = order_schema.OrderCreate(
            description="Customer cart",
            status=order_schema.StatusType.PENDING,
            order_type=order_schema.OrderType.DINE_IN
        )
        new_order = order_controller.create(db=db, request=order_request)
        order_id = new_order.id
    
    # make sure menu item exists
    menu_item = db.query(MenuItem).filter(MenuItem.id == menu_item_id).first()
    if not menu_item:
        raise HTTPException(status_code=404, detail="Menu item not found")
    
    # get the ingredients for the item
    required_ingredients = db.query(MenuItemIngredient).filter(
        MenuItemIngredient.menu_item_id == menu_item_id
    ).all()
    
    max_possible = float('inf')
    
    # Calculate how many can be made
    for ingredient in required_ingredients:
        resource = db.query(Resource).filter(Resource.id == ingredient.resource_id).first()
        if resource:
            possible_quantity = resource.amount // ingredient.amount
            max_possible = min(max_possible, possible_quantity)
    
    # Check if item already exists in cart
    existing_item = db.query(OrderDetail).filter(
        OrderDetail.order_id == order_id,
        OrderDetail.menu_item_id == menu_item_id
    ).first()
    
    if existing_item:
        # Item exists, check if new total quantity is available
        new_total_quantity = existing_item.amount + quantity
        if new_total_quantity > max_possible:
            raise HTTPException(
                status_code=400, 
                detail=f"Cannot add {quantity} {menu_item.name}, max is {max_possible} (you currently have {existing_item.amount} in cart)"
            )
        
        # Update the cart
        existing_item.amount = new_total_quantity
        db.commit()
        db.refresh(existing_item)
        return {
            "order_detail": existing_item,
            "order_id": order_id
        }
    else:
        # Add a new menu item
        if quantity > max_possible:
            raise HTTPException(
                status_code=400, 
                detail=f"Cannot add {quantity} {menu_item.name}, max is {max_possible}"
            )
        
        # Add new item to the order
        order_detail_request = order_detail_schema.OrderDetailCreate(
            order_id=order_id,
            menu_item_id=menu_item_id,
            amount=quantity
        )
        order_detail = order_detail_controller.create(db=db, request=order_detail_request)
        return {
            "order_detail": order_detail,
            "order_id": order_id
        }


def remove_item_from_cart(db: Session, order_id: int, menu_item_id: int):
    """
    Delete an item from the cart
    """
    from ..models.order_details import OrderDetail
    from fastapi import HTTPException
    
    # Find the item in the cart
    item = db.query(OrderDetail).filter(
        OrderDetail.order_id == order_id,
        OrderDetail.menu_item_id == menu_item_id
    ).first()
    
    if not item:
        raise HTTPException(status_code=404, detail="Item not found in cart")
    
    # Get the menu item name before deleting
    menu_item_name = item.menu_item.name
    
    # Delete the item
    db.delete(item)
    db.commit()
    
    return {"message": f"{menu_item_name} removed from cart"}


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


def process_payment(db: Session, order: Order):
    # placeholder logic, will not be implemented
    # get payment info from the order id
    from ..models.payment_method import Payment
    from ..schemas.payment_method import PaymentType
    
    payment = order.payment
    if not payment:
        raise HTTPException(status_code=400, detail="No payment method found for this order")


    example_response = {
        "status": "approved", # assume payment was succcessful
        "transaction_id": "txn_ABC123456789"
    }

    if order.payment.payment_type != PaymentType.CASH:
        # "send " payment to processor
        # wait for a response
        response = example_response

         # handle response

    return True


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
    
    # For each menu item in the order
    for detail in order_details:
        menu_item_id = detail.menu_item_id
        quantity_ordered = detail.amount
        
        # Get required ingredients for this menu item
        required_ingredients = db.query(MenuItemIngredient).filter(
            MenuItemIngredient.menu_item_id == menu_item_id
        ).all()
        
        # update each ingredient in  stock
        for ingredient in required_ingredients:
            resource = db.query(Resource).filter(Resource.id == ingredient.resource_id).first()
            
            if resource:
                # Calculate total amount needed (ingredient amount * quantity ordered)
                total_needed = ingredient.amount * quantity_ordered
                
                # if 2 customers are ordering at the same time, the may be able to add
                # an item when there are insufficient resources
                if resource.amount < total_needed:
                    raise HTTPException(
                        status_code=400, 
                        detail=f"Insufficient {resource.item} inventory. Need {total_needed}, have {resource.amount}"
                    )
                
                resource.amount -= total_needed
    
    # Commit all changes
    db.commit()
    return True


def generate_tracking_number():
    """
    generate an 8-digit random tracking number
    """
    return ''.join(random.choices(string.ascii_uppercase + string.digits, k=8))


def checkout(db: Session, order_id: int, response=None):
    """
    Process checkout for an order
    """
    from ..models.promotions import Promotion
    from ..models.orders import Order
    from datetime import datetime
    from fastapi import HTTPException
    from ..schemas.orders import StatusType
    from ..schemas.orders import OrderUpdate

    # Get the order
    order = db.query(Order).filter(Order.id == order_id).first()
    if not order:
        raise HTTPException(status_code=404, detail="Order not found")

    # check if the order has payment
    if not order.payment:
        raise HTTPException(status_code=400, detail="No payment method found. Please add a payment method before checkout.")

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
    
    # process the payment
    payment_result = process_payment(db, order)
    
    #change order status to paid
    try:
        order_update = OrderUpdate(paid=True)
        updated_order = order_controller.update(db=db, request=order_update, item_id=order_id)
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to update order payment status: {str(e)}")

    # Change order to in progress
    try:
        order_update = OrderUpdate(status=StatusType.IN_PROGRESS)
        updated_order = order_controller.update(db=db, request=order_update, item_id=order_id)
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to update order status to in progress: {str(e)}")

    # Update (deduct) raw ingredients
    if not update_raw_ingredients(db, order_id):
        raise HTTPException(status_code=500, detail="Failed to update stock")

    # the "order date" and time need to be updated at checkout
    try:
        order_update = OrderUpdate(order_date=datetime.now())
        order_controller.update(db=db, request=order_update, item_id=order_id)
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to update order date: {str(e)}")

    # Clear the browser cookie after successful checkout
    if response:
        response.delete_cookie(key="order_id")
    
    # Update menu if insufficient ingredients 
    # Send to kitchen (print ticket)

    # if takeout or delivery provide tracking number
    tracking_number = None
    # Check order type using string values to avoid enum comparison issues
    if order.order_type.value in ["takeout", "delivery"]:
        # only generate if no existing number
        if not order.tracking_number:
            tracking_number = generate_tracking_number()
            
            # check for duplicate tracking number
            while db.query(Order).filter(Order.tracking_number == tracking_number).first():
                tracking_number = generate_tracking_number()
            
            # update the order with the tracking number
            try:
                order_update = OrderUpdate(tracking_number=tracking_number)
                updated_order = order_controller.update(db=db, request=order_update, item_id=order_id)
                # Refresh the order object to get the updated tracking number
                db.refresh(order)
            except Exception as e:
                raise HTTPException(status_code=500, detail=f"Failed to update tracking number: {str(e)}")
        else:
            tracking_number = order.tracking_number
    
    # Prepare response message
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
    
    # Return tracking information
    return {
        "tracking_number": tracking_number,
        "order_id": order.id,
        "status": order.status.value,
        "order_type": order.order_type.value,
        "order_date": order.order_date,
        "paid": order.paid
    }


def choose_order_type(db: Session, order_id: int, type):
    """
    Update the order type (dine_in, takeout, delivery)
    Tracking number will be generated at checkout for takeout and delivery orders
    """
    from ..schemas.orders import OrderUpdate
    from ..controllers import orders as order_controller
    
    # Create the order update with the new type
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