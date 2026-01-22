from sqlalchemy.orm import Session
from shared.repositories import BaseRepository
from ..models.promotions import Promotion
from ..schemas.promotions import PromotionCreate, PromotionUpdate

# Initialize repository
promotion_repo = BaseRepository[Promotion, PromotionCreate, PromotionUpdate](Promotion)


def create(db: Session, request: PromotionCreate):
    return promotion_repo.create(db, request)


def read_all(db: Session):
    return promotion_repo.get_all(db)


def read_one(db: Session, item_id: int):
    return promotion_repo.get_or_404(db, item_id)


def update(db: Session, item_id: int, request: PromotionUpdate):
    return promotion_repo.update(db, item_id, request)


def delete(db: Session, item_id: int):
    promotion_repo.delete(db, item_id)