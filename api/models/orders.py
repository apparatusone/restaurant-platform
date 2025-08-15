from sqlalchemy import Column, ForeignKey, Integer, String, DateTime, Enum, Text
from sqlalchemy.orm import relationship
from sqlalchemy.sql import func
import enum
from ..dependencies.database import Base


class OrderType(enum.Enum):
    DINE_IN = "dine_in"
    TAKEOUT = "takeout"
    DELIVERY = "delivery"


class OrderStatus(enum.Enum):
    # kitchen workflow
    PENDING = "pending"
    CONFIRMED = "confirmed"
    IN_PROGRESS = "in_progress"
    READY = "ready"
    
    # fulfillment workflow
    AWAITING_PICKUP = "awaiting_pickup"
    OUT_FOR_DELIVERY = "out_for_delivery"
    DELIVERED = "delivered"
    SERVED = "served"  # for dine-in
    COMPLETED = "completed"
    CANCELLED = "cancelled"


class Order(Base):
    __tablename__ = "orders"

    id = Column(Integer, primary_key=True, autoincrement=True)
    check_id = Column(Integer, ForeignKey("checks.id"), nullable=False)
    customer_id = Column(Integer, ForeignKey("customers.id"), nullable=True)
    promo_id = Column(Integer, ForeignKey("promotions.id"), nullable=True)
    status = Column(Enum(OrderStatus), default=OrderStatus.PENDING, nullable=False)
    order_type = Column(Enum(OrderType), default=OrderType.DINE_IN, nullable=False)
    tracking_number = Column(String(50), nullable=True, unique=True, index=True)
    notes = Column(Text, nullable=True)
    
    # timestamps
    order_date = Column(DateTime(timezone=True), server_default=func.now(), nullable=False)
    created_at = Column(DateTime(timezone=True), server_default=func.now(), nullable=False)
    updated_at = Column(DateTime(timezone=True), server_default=func.now(), onupdate=func.now(), nullable=False)


    # relationships
    check = relationship("Check", back_populates="orders")
    customer = relationship("Customer", back_populates="orders")
    promo = relationship("Promotion", back_populates="orders")
    order_details = relationship("OrderDetail", back_populates="order", cascade="all, delete-orphan")
    payments = relationship("Payment", back_populates="order")