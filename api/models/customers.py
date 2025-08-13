from sqlalchemy import Column, Integer, String, BigInteger
from sqlalchemy.orm import relationship
from ..dependencies.database import Base

class Customer(Base):
    __tablename__ = "customers"

    id = Column(Integer, primary_key=True, index=True, autoincrement=True)
    customer_name = Column(String(100))
    customer_email = Column(String(100))
    customer_phone = Column(BigInteger, index=True, nullable=True)  # Changed to BigInteger to handle 10-digit phone numbers
    customer_address = Column(String(100))

    orders = relationship("Order", back_populates="customer")
