import socket
from datetime import datetime
from typing import Optional
from api.config.restaurant import (
    RESTAURANT_NAME, 
    ADDRESS as RESTAURANT_ADDRESS, 
    PHONE as RESTAURANT_PHONE, 
    TAX_RATE
)
from api.dependencies.database import get_db
from api.controllers.orders import read_one as get_order_by_id
from api.controllers import order_details as order_detail_controller

SERVER_IP = "10.0.1.43"
PORT = 9100

ESC, GS = b"\x1b", b"\x1d"
INIT     = ESC+b"@"
ALIGN_L  = ESC+b"a\x00"
ALIGN_C  = ESC+b"a\x01"
ALIGN_R  = ESC+b"a\x02"
BOLD_ON  = ESC+b"E\x01"
BOLD_OFF = ESC+b"E\x00"
SIZE_N   = GS+b"!\x00"  # Normal size
SIZE_L   = GS+b"!\x11"  # Double width and height
CUT_FULL = GS+b"V\x00"

COLS = 32  # 58mm = ~32 columns in Font A
HR   = b"-"*COLS + b"\n"

def get_order_items_for_receipt(order_id: int) -> list[tuple[str, float]]:
    """
    Get order items formatted for receipt printing.
    Returns list of tuples: (formatted_item_name, price)
    """
    from api.models.order_details import OrderDetail
    from api.models.menu_items import MenuItem
    
    db = next(get_db())
    
    # Query order details with menu item information
    order_details = db.query(OrderDetail).join(MenuItem).filter(
        OrderDetail.order_id == order_id
    ).all()
    
    items = []
    for detail in order_details:
        # format item name: "QUANTITY x ITEM_NAME"
        item_name = detail.menu_item.name.upper()
        
        # calculate max length for item name
        # Format: "QTY x ITEM_NAME" + space + "PRICE"
        # item, quantity, " x ", space, and price (up to 8 chars)
        max_item_length = COLS - len(str(detail.amount)) - 3 - 1 - 8
        
        # truncate item name
        if len(item_name) > max_item_length:
            item_name = item_name[:max_item_length-3] + "..."
        
        formatted_name = f"{detail.amount} x {item_name}"
        
        # Use the price from menu item (current price)
        price = float(detail.menu_item.price)
        
        items.append((formatted_name, price))
    
    return items
        
def print_receipt(order_id: int, subtotal: float = None, tax: float = None, total: float = None):
    def format_line(label: str, price: Optional[float] = None) -> bytes:
        if price is None:
            formatted_label = label[:COLS].ljust(COLS)
            return formatted_label.encode("ascii", "ignore") + b"\n"
        
        # ensure price is a float
        try:
            price_float = float(price)
            price_str = f"{price_float:,.2f}"
        except (ValueError, TypeError):
            print(f"Debug: Invalid price value: {price} (type: {type(price)})")
          
        
        # calculate available width
        available_width = COLS - len(price_str) - 1
        
        # format label and price with alignment
        formatted_label = label[:available_width].ljust(available_width)
        formatted_line = formatted_label.encode("ascii", "ignore") + b" " + price_str.encode() + b"\n"
        
        return formatted_line

    def send(data: bytes):
        with socket.create_connection((SERVER_IP, PORT), timeout=5) as s:
            s.sendall(data)

    # get items from the order
    items = get_order_items_for_receipt(order_id)
    
    # get order info for promo code
    from api.models.orders import Order
    from api.models.promotions import Promotion
    db = next(get_db())
    order = db.query(Order).filter(Order.id == order_id).first()
    order_type = order.order_type.value.upper() if order else ""
    time = order.order_date.strftime("%H:%M")

    # reciept line with order id and time:
    order_left = f"ORDER: {order_id}"
    spacing = " " * (COLS - len(order_left) - len(time))
    order_line = f"{order_left}{spacing}{time}"
    
    # get promo info if exists
    promo_code = None
    promo_discount = 0
    if order and order.promo_id:
        promotion = db.query(Promotion).filter(Promotion.id == order.promo_id).first()
        if promotion:
            promo_code = promotion.code
            # calculate discount amount
            original_subtotal = sum(p for _, p in items)
            promo_discount = original_subtotal * (promotion.discount_percent / 100)

    # use values or calculate if not provided
    if subtotal is None or tax is None or total is None:
        calculated_subtotal = sum(p for _, p in items)
        calculated_tax = calculated_subtotal * TAX_RATE
        calculated_total = calculated_subtotal + calculated_tax
        
        subtotal = subtotal or calculated_subtotal
        tax = tax or calculated_tax
        total = total or calculated_total

    buf  = INIT + ALIGN_C + SIZE_L + BOLD_ON
    buf += RESTAURANT_NAME.encode("ascii", "ignore") + b"\n"
    buf += BOLD_OFF + SIZE_N
    buf += RESTAURANT_ADDRESS.encode("ascii", "ignore") + b"\n"
    buf += HR
    buf += ALIGN_L + BOLD_ON + order_line.encode("ascii", "ignore") + b"\n" + BOLD_OFF
    buf += HR
    for n,p in items: buf += format_line(n, p)
    buf += HR
    buf += format_line("SUBTOTAL", subtotal + promo_discount if promo_discount > 0 else subtotal)
    if promo_code:
        buf += format_line(f"PROMO: {promo_code}", -promo_discount)
    buf += format_line("TAX", tax)
    buf += BOLD_ON + format_line("TOTAL", total) + BOLD_OFF + b"\n"
    buf += ALIGN_C
    buf += b"THANK YOU!\n"
    buf += b"CUSTOMER COPY"
    buf += b"\n"*3 + CUT_FULL

    send(buf)