from sqlalchemy.orm import Session
from shared.repositories import BaseRepository
from ..models.customers import Customer
from ..schemas.customers import CustomerCreate, CustomerUpdate

# Initialize repository
customer_repo = BaseRepository[Customer, CustomerCreate, CustomerUpdate](Customer)


def create(db: Session, request: CustomerCreate):
    return customer_repo.create(db, request)


def read_all(db: Session):
    return customer_repo.get_all(db)


def read_one(db: Session, item_id: int):
    return customer_repo.get_or_404(db, item_id)


def update(db: Session, item_id: int, request: CustomerUpdate):
    return customer_repo.update(db, item_id, request)


def delete(db: Session, item_id: int):
    customer_repo.delete(db, item_id)
