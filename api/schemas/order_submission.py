from pydantic import BaseModel
from typing import Optional, List
from .orders import OrderType


class OrderItemSubmission(BaseModel):
    """Individual item in the order submission"""
    menu_item_id: int
    quantity: int
    price: float


class CustomerInfoSubmission(BaseModel):
    """Customer information from the checkout form"""
    name: str
    email: str
    phone: str


class DeliveryInfoSubmission(BaseModel):
    """Delivery information from the checkout form"""
    address: str
    city: str
    zipCode: str
    instructions: Optional[str] = None


class PaymentInfoSubmission(BaseModel):
    """Payment information from the checkout form (for demo purposes)"""
    cardNumber: str
    expiryDate: str
    cvv: str
    nameOnCard: str


class CompleteOrderSubmission(BaseModel):
    """Complete order submission from the browser"""
    # Order items
    items: List[OrderItemSubmission]
    total_price: float
    item_count: int
    
    customer: CustomerInfoSubmission
    delivery: DeliveryInfoSubmission
    
    # Payment information
    payment: PaymentInfoSubmission
    
    # Order metadata
    timestamp: str
    order_type: OrderType = OrderType.DELIVERY


class OrderSubmissionResponse(BaseModel):
    """Response after successful order submission"""
    success: bool
    message: str
    order_id: int
    tracking_number: str
    estimated_delivery_time: Optional[str] = None