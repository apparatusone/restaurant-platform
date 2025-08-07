#!/usr/bin/env python3
"""
Script to add test order data with order details and payments
"""

from api.dependencies.database import SessionLocal
from api.models.orders import Order, OrderType, StatusType
from api.models.order_details import OrderDetail
from api.models.payment_method import Payment, PaymentType, PaymentStatus
from api.models.menu_items import MenuItem
from api.models.customers import Customer
from api.models import model_loader
from datetime import datetime


def add_test_orders():
    model_loader.index()
    
    db = SessionLocal()
    
    try:
        # check if there are enough menu items and customers
        menu_items = db.query(MenuItem).limit(4).all()
        customers = db.query(Customer).limit(4).all()
        
        if len(menu_items) < 3:
            print("Not enough menu items found. Please run add_test_food.py first.")
            return False
            
        if len(customers) < 3:
            print("Not enough customers found. Please run add_test_customers.py first.")
            return False
        
        orders_data = [
            {
                "customer_id": customers[0].id,
                "description": "Example order 1",
                "status": StatusType.COMPLETED,
                "order_type": OrderType.DINE_IN,
                "paid": True,
                "tracking_number": None,
                "payment_type": PaymentType.CASH,
                "final_total": 34.12,
                "order_items": [
                    {"menu_item_id": menu_items[0].id, "amount": 1},
                    {"menu_item_id": menu_items[1].id, "amount": 1}
                ]
            },
            {
                "customer_id": customers[1].id,
                "description": "Example order 2",
                "status": StatusType.AWAITING_PICKUP,
                "order_type": OrderType.TAKEOUT,
                "paid": True,
                "tracking_number": "TK123ABC",
                "payment_type": PaymentType.CREDIT_CARD,
                "card_number": "4532123456789012",
                "final_total": 14.12,
                "order_items": [
                    {"menu_item_id": menu_items[0].id, "amount": 2}
                ]
            },
            {
                "customer_id": customers[2].id,
                "description": "Example order 3",
                "status": StatusType.OUT_FOR_DELIVERY,
                "order_type": OrderType.DELIVERY,
                "paid": True,
                "tracking_number": "DL456XYZ",
                "payment_type": PaymentType.CREDIT_CARD,
                "final_total": 94.12,
                "card_number": "4532987654321098",
                "order_items": [
                    {"menu_item_id": menu_items[0].id, "amount": 1},
                    {"menu_item_id": menu_items[2].id, "amount": 2}
                ]
            }
        ]
        
        created_orders = []
        
        for order_data in orders_data:
            # Create the order
            order = Order(
                customer_id=order_data["customer_id"],
                description=order_data["description"],
                status=order_data["status"],
                order_type=order_data["order_type"],
                paid=order_data["paid"],
                final_total=order_data["final_total"],
                tracking_number=order_data["tracking_number"],
                order_date=datetime.now()
            )
            
            db.add(order)
            db.flush()
            
            # create order details
            for item_data in order_data["order_items"]:
                order_detail = OrderDetail(
                    order_id=order.id,
                    menu_item_id=item_data["menu_item_id"],
                    amount=item_data["amount"]
                )
                db.add(order_detail)
            
            # create payment
            payment = Payment(
                order_id=order.id,
                payment_date=datetime.now(),
                status=PaymentStatus.COMPLETED,
                payment_type=order_data["payment_type"],
                card_number=order_data.get("card_number")
            )
            db.add(payment)
            
            created_orders.append({
                "id": order.id,
                "customer_id": order.customer_id,
                "description": order.description,
                "status": order.status.value,
                "order_type": order.order_type.value,
                "tracking_number": order.tracking_number,
                "items_count": len(order_data["order_items"]),
                "payment_type": order_data["payment_type"].value
            })
        
        db.commit()
        print(f"Successfully created {len(created_orders)} test orders with order details and payments")
        return True
        
    except Exception as e:
        db.rollback()
        print(f"Error adding test orders: {e}")
        return False
    
    finally:
        db.close()


if __name__ == "__main__":
    add_test_orders()