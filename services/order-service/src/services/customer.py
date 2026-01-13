import random
import string
import logging
from datetime import datetime
from sqlalchemy.orm import Session
from fastapi import HTTPException
from typing import Optional
from sqlalchemy.exc import SQLAlchemyError
from ..utils.errors import (
    handle_sqlalchemy_error,
    raise_not_found,
    DatabaseError
)
from shared.models.menu_items import MenuItem
from ..models.menu_item_ingredients import MenuItemIngredient
from ..models.resources import Resource
from shared.schemas import orders as order_schema
from shared.schemas import order_items as order_item_schema
from ..schemas import payment_method as payment_schema
from ..schemas import customers as customer_schema
from ..controllers import orders as order_controller
from ..controllers import order_items as order_item_controller
from ..controllers import payment_method as payment_controller
from ..controllers import customers as customer_controller
from ..schemas.payment_method import PaymentType
from shared.models.order_items import OrderItem
from shared.models.menu_items import MenuItem
from .print import print_receipt
from shared.models.orders import Order

from decimal import Decimal

logger = logging.getLogger(__name__)
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
    # Get all order items for this order
    order_items = db.query(OrderItem).filter(OrderItem.order_id == order_id).all()
    
    if not order_items:
        return True, "Order is empty"
    
    total_ingredient_needs = {}
    
    for order_item in order_items:
        # get required ingredients
        ingredients = db.query(MenuItemIngredient).filter(
            MenuItemIngredient.menu_item_id == order_item.menu_item_id
        ).all()
        
        for ingredient in ingredients:
            resource_id = ingredient.resource_id
            needed_amount = ingredient.amount * order_item.quantity
            
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
        raise_not_found("Menu item", item_name)
    
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
    from shared.models.orders import Order
    from shared.schemas.orders import OrderUpdate
    from ..controllers import orders as order_controller
    from fastapi import HTTPException, status
    from datetime import datetime
    
    promo = db.query(Promotion).filter(Promotion.code == promo_code).first()
    
    if not promo:
        raise_not_found("Promo code", promo_code)
    
    # check if promo expired
    if promo.expiration_date and promo.expiration_date < datetime.now():
        raise HTTPException(status_code=400, detail="Promo code has expired")
    
    # check that the order exists
    order = db.query(Order).filter(Order.id == order_id).first()
    if not order:
        raise_not_found("Order", order_id)
    
    order_update = OrderUpdate(promo_id=promo.id)
    updated_order = order_controller.update(db=db, request=order_update, item_id=order_id)
    
    return {"message": f"Promo code '{promo_code}' applied successfully"}


def calculate_order_total(db: Session, order_id: int) -> float:
    """
    Calculate the total amount for an order
    """
    # Get all order items on an order
    order_items = db.query(OrderItem).join(MenuItem).filter(
        OrderItem.order_id == order_id
    ).all()
    
    total = Decimal('0.00')
    
    for item in order_items:
        # handle data types
        item_total = Decimal(item.quantity) * item.menu_item.price
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
    from shared.models.orders import Order
    from shared.models.order_items import OrderItem
    from ..models.menu_item_ingredients import MenuItemIngredient
    from ..models.resources import Resource
    
    order_items = db.query(OrderItem).filter(OrderItem.order_id == order_id).all()
    
    if not order_items:
        raise_not_found("Order items", order_id)
    
    try:
        for item in order_items:
            menu_item_id = item.menu_item_id
            quantity_ordered = item.quantity
            
            required_ingredients = db.query(MenuItemIngredient).filter(
                MenuItemIngredient.menu_item_id == menu_item_id
            ).all()
            
            # update each ingredient in  stock
            for ingredient in required_ingredients:
                resource = db.query(Resource).filter(Resource.id == ingredient.resource_id).first()
                
                if resource:
                    total_needed = ingredient.amount * quantity_ordered
                    
                    if resource.amount < total_needed:
                        from ..utils.errors import ValidationError
                        ValidationError(
                            f"Insufficient {resource.item} inventory. Need {total_needed}, have {resource.amount}"
                        ).raise_exception()
                    
                    resource.amount -= total_needed
        
        db.commit()
    except SQLAlchemyError as e:
        db.rollback()
        handle_sqlalchemy_error(e).raise_exception()
    except Exception:
        db.rollback()
        DatabaseError("Failed to update inventory").raise_exception()
        
    return True





# TODO: this handles far too much, simplify
def checkout(db: Session, order_id: int, response=None):
    """
    Process checkout for an order
    """
    from ..models.promotions import Promotion
    from shared.models.orders import Order
    from datetime import datetime
    from fastapi import HTTPException
    from shared.schemas.orders import OrderStatus
    from shared.schemas.orders import OrderUpdate
    from ..config.restaurant import TAX_RATE

    # Get the order
    order = db.query(Order).filter(Order.id == order_id).first()
    if not order:
        raise_not_found("Order", order_id)

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
            
    except SQLAlchemyError as e:
        handle_sqlalchemy_error(e).raise_exception()
    except Exception:
        DatabaseError("Failed to update order payment status").raise_exception()

    # change order status
    try:
        order_update = OrderUpdate(status=OrderStatus.IN_PROGRESS)
        updated_order = order_controller.update(db=db, request=order_update, item_id=order_id)
    except SQLAlchemyError as e:
        handle_sqlalchemy_error(e).raise_exception()
    except Exception:
        DatabaseError("Failed to update order status").raise_exception()

    # update resources
    if not update_raw_ingredients(db, order_id):
        raise HTTPException(status_code=500, detail="Failed to update stock")

    # the "order date" and time need to be updated at checkout
    try:
        order_update = OrderUpdate(order_date=datetime.now())
        order_controller.update(db=db, request=order_update, item_id=order_id)
    except SQLAlchemyError as e:
        handle_sqlalchemy_error(e).raise_exception()
    except Exception:
        DatabaseError("Failed to update order date").raise_exception()

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
            logger.info("Email receipt should be sent for delivery order", extra={"order_id": order_id})
    except Exception as e:
        # Log the error but don't fail the checkout
        logger.warning("Failed to process receipt", extra={"order_id": order_id}, exc_info=True) 

    # if takeout or delivery provide tracking number
    tracking_number = None
    # Check order type using string values to avoid enum comparison issues
    if order.order_type.value in ["takeout", "delivery"]:
        # only generate if no existing number
        if not order.tracking_number:
            from ..controllers.orders import generate_tracking_number
            from shared.models.orders import OrderType
            
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
            except SQLAlchemyError as e:
                handle_sqlalchemy_error(e).raise_exception()
            except Exception:
                DatabaseError("Failed to update tracking number").raise_exception()
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
    from shared.models.orders import Order
    from fastapi import HTTPException
    
    # Find the order by tracking number
    order = db.query(Order).filter(Order.tracking_number == tracking_number).first()
    
    if not order:
        raise_not_found("Tracking number", tracking_number)
    
    return {
        "tracking_number": tracking_number,
        "status": order.status.value
    }


def choose_order_type(db: Session, order_id: int, type):
    """
    Update the order type (dine_in, takeout, delivery)
    Tracking number will be generated at checkout for takeout and delivery orders
    """
    from shared.schemas.orders import OrderUpdate
    from ..controllers import orders as order_controller
    
    # create the order update with the new type
    order_update = OrderUpdate(order_type=type)
    
    return order_controller.update(db=db, request=order_update, item_id=order_id)


def get_reviews(db: Session):
    """
    Get menu items that have reviews and the reviews
    """
    from shared.models.menu_items import MenuItem
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


def submit_complete_order(db: Session, order_data):
    """
    Process a complete order submission from the browser
    Creates customer, order, order details, and payment
    Validates all prices and data integrity server-side
    """
    from ..schemas.order_submission import CompleteOrderSubmission
    from ..models.customers import Customer
    from shared.models.orders import Order
    from shared.models.order_items import OrderItem
    from ..models.payment_method import Payment
    from shared.schemas.orders import OrderStatus
    from ..schemas.payment_method import PaymentType
    from ..config.restaurant import TAX_RATE
    import uuid

    logger.debug("Starting order submission validation")
    
    try:
        # Validate order items and calculate server-side totals
        server_subtotal = Decimal('0.00')
        validated_items = []
        
        for item in order_data.items:
            # Verify menu item exists and get current price
            menu_item = db.query(MenuItem).filter(MenuItem.id == item.menu_item_id).first()
            if not menu_item:
                raise HTTPException(status_code=400, detail=f"Menu item {item.menu_item_id} not found")
            
            # Validate quantity is positive
            if item.quantity <= 0:
                raise HTTPException(status_code=400, detail=f"Invalid quantity {item.quantity} for item {menu_item.name}")
            
            # Calculate server-side price (don't trust client price)
            server_item_total = Decimal(str(menu_item.price)) * Decimal(str(item.quantity))
            server_subtotal += server_item_total
            
            # Compare with client-submitted price (allow small floating point differences)
            client_item_total = Decimal(str(item.price)) * Decimal(str(item.quantity))
            price_difference = abs(server_item_total - client_item_total)
            
            if price_difference > Decimal('0.01'):  # Allow 1 cent difference for floating point
                raise HTTPException(
                    status_code=400, 
                    detail=f"Price mismatch for {menu_item.name}. Expected {float(server_item_total)}, got {float(client_item_total)}"
                )
            
            validated_items.append({
                'menu_item': menu_item,
                'quantity': item.quantity,
                'server_price': server_item_total
            })
        
        # Calculate server-side tax and total
        server_tax = server_subtotal * Decimal(str(TAX_RATE))
        
        # Add delivery fee for delivery orders
        delivery_fee = Decimal('0.00')
        if hasattr(order_data, 'order_type') and order_data.order_type == 'delivery':
            from ..config.restaurant import CONFIG
            delivery_fee = Decimal(str(CONFIG["business_settings"].get("delivery_fee", 3.99)))
        
        server_total = server_subtotal + server_tax + delivery_fee
        
        # Validate client total (allow small floating point differences)
        client_total = Decimal(str(order_data.total_price))
        total_difference = abs(server_total - client_total)
        
        if total_difference > Decimal('0.01'):  # Allow 1 cent difference
            raise HTTPException(
                status_code=400,
                detail=f"Order total mismatch. Expected {float(server_total)}, got {float(client_total)}"
            )
        
        # Validate item count
        if order_data.item_count != sum(item.quantity for item in order_data.items):
            raise HTTPException(
                status_code=400,
                detail="Item count mismatch"
            )
        
        customer_data = order_data.customer
        
        # Validate customer data
        if not customer_data.name or len(customer_data.name.strip()) < 2:
            raise HTTPException(status_code=400, detail="Customer name must be at least 2 characters")
        
        if not customer_data.email or '@' not in customer_data.email:
            raise HTTPException(status_code=400, detail="Valid email address required")
        
        # Validate and clean phone number
        clean_phone = customer_data.phone.replace('(', '').replace(')', '').replace('-', '').replace(' ', '').replace('+1', '')
        if not clean_phone.isdigit() or len(clean_phone) != 10:
            raise HTTPException(status_code=400, detail="Phone number must be 10 digits")
        
        # Validate delivery address
        if not order_data.delivery.address or len(order_data.delivery.address.strip()) < 5:
            raise HTTPException(status_code=400, detail="Delivery address must be at least 5 characters")
        
        if not order_data.delivery.city or len(order_data.delivery.city.strip()) < 2:
            raise HTTPException(status_code=400, detail="City must be at least 2 characters")
        
        if not order_data.delivery.zipCode or not order_data.delivery.zipCode.replace('-', '').isdigit():
            raise HTTPException(status_code=400, detail="Valid ZIP code required")
        
        # check if customer already exists by email
        existing_customer = db.query(Customer).filter(Customer.customer_email == customer_data.email.lower().strip()).first()
        
        if existing_customer:
            customer = existing_customer
            # Update existing customer with new information if provided
            customer.customer_name = customer_data.name.strip()
            customer.customer_phone = int(clean_phone)
            customer.customer_address = f"{order_data.delivery.address.strip()}, {order_data.delivery.city.strip()}, {order_data.delivery.zipCode.strip()}"
        else:
            # create new customer
            customer = Customer(
                customer_name=customer_data.name.strip(),
                customer_email=customer_data.email.lower().strip(),
                customer_phone=int(clean_phone),
                customer_address=f"{order_data.delivery.address.strip()}, {order_data.delivery.city.strip()}, {order_data.delivery.zipCode.strip()}"
            )
            db.add(customer)
            db.flush()
        
        # Generate unique tracking number
        from ..controllers.orders import generate_tracking_number
        tracking_number = generate_tracking_number(order_data.order_type)
        
        # Check for duplicate tracking numbers and regenerate if needed
        while db.query(Order).filter(Order.tracking_number == tracking_number).first():
            tracking_number = generate_tracking_number(order_data.order_type)
        
        order = Order(
            customer_id=customer.id,
            description=f"Online order - {len(validated_items)} items",
            status=OrderStatus.CONFIRMED,
            order_type=order_data.order_type,
            paid=True,  # Assuming payment is processed (demo)
            final_total=float(server_total),  # Use server-calculated total
            tracking_number=tracking_number,
            order_date=datetime.now()
        )
        db.add(order)
        db.flush()
        
        # create order details using validated items
        for validated_item in validated_items:
            menu_item = validated_item['menu_item']
            quantity = validated_item['quantity']
            

            order_item = OrderItem(
                order_id=order.id,
                menu_item_id=menu_item.id,
                quantity=quantity,
                unit_price=menu_item.price,
                line_total=menu_item.price * quantity
            )
            db.add(order_item)
        
        # Flush order details to make them available for the resource check
        db.flush()
        
        # Validate payment data (basic validation for demo)
        if not order_data.payment.cardNumber or len(order_data.payment.cardNumber.replace(' ', '').replace('-', '')) < 13:
            raise HTTPException(status_code=400, detail="Valid credit card number required")
        
        if not order_data.payment.expiryDate or len(order_data.payment.expiryDate) < 5:
            raise HTTPException(status_code=400, detail="Valid expiry date required (MM/YY)")
        
        if not order_data.payment.cvv or not order_data.payment.cvv.isdigit() or len(order_data.payment.cvv) < 3:
            raise HTTPException(status_code=400, detail="Valid CVV required")
        
        if not order_data.payment.nameOnCard or len(order_data.payment.nameOnCard.strip()) < 2:
            raise HTTPException(status_code=400, detail="Name on card required")
        
        # create payment record (demo - not actually processing payment)
        clean_card_number = order_data.payment.cardNumber.replace(' ', '').replace('-', '')
        payment = Payment(
            order_id=order.id,
            payment_type=PaymentType.CREDIT_CARD,
            card_number=clean_card_number[-4:],  # Only store last 4 digits
        )
        db.add(payment)
        db.flush()
        
        # Verify order items were created
        order_items_count = db.query(OrderItem).filter(OrderItem.order_id == order.id).count()

        # check if order can be made (resource availability)
        can_make, message = can_order_be_made(db, order.id)
        
        if can_make:
            # Order can be made - deduct resources and confirm
            try:
                update_raw_ingredients(db, order.id)
                order.status = OrderStatus.CONFIRMED
                db.commit()
                
                # print receipt for confirmed orders
                try:
                    print_receipt(db, order.id)
                except Exception as e:
                    logger.error("Failed to print receipt", extra={"order_id": order.id}, exc_info=True)
                    
            except HTTPException as resource_error:
                # Resources became unavailable between check and deduction
                db.rollback()
                order.status = OrderStatus.PENDING
                order.description += f" - Resource shortage: {resource_error.detail}"
                can_make = False
                message = resource_error.detail
                db.commit()
        else:
            # Order cannot be made due to insufficient resources
            order.status = OrderStatus.PENDING
            order.description += f" - Insufficient resources: {message}"
            db.commit()
        
        # Format message based on order status
        if can_make:
            response_message = "Order placed successfully! Your order is being prepared."
            delivery_time = "30-45 minutes"
        else:
            # Handle resource shortage message
            if isinstance(message, list):
                # message is a list of missing ingredients
                missing_items = [f"{item['resource_name']} (need {item['needed']}, have {item['available']})" 
                               for item in message]
                response_message = f"Order received but cannot be fulfilled due to insufficient ingredients: {', '.join(missing_items)}. We'll prepare your order as soon as ingredients are restocked."
            else:
                # message is a string
                response_message = f"Order received but cannot be fulfilled: {message}. We'll prepare your order as soon as possible."
            delivery_time = "Pending ingredient availability"

        logger.debug(
            "Order processed successfully",
            extra={
                "order_id": order.id,
                "status": order.status,
                "tracking_number": tracking_number
            }
        )

        return {
            "success": True,
            "message": response_message,
            "order_id": order.id,
            "tracking_number": tracking_number,
            "estimated_delivery_time": delivery_time,
            "order_status": "confirmed" if can_make else "pending_resources",
            "final_total": float(server_total),  # Return server-calculated total
            "subtotal": float(server_subtotal),
            "tax": float(server_tax)
        }
        
    except SQLAlchemyError as e:
        db.rollback()
        handle_sqlalchemy_error(e).raise_exception()
    except Exception:
        db.rollback()
        DatabaseError("Failed to process order").raise_exception()