from sqlalchemy.orm import Session
from shared.repositories import BaseRepository
from ..models.reviews import Reviews
from ..schemas.reviews import ReviewsCreate, ReviewsUpdate

# Initialize repository
review_repo = BaseRepository[Reviews, ReviewsCreate, ReviewsUpdate](Reviews)


def create(db: Session, request: ReviewsCreate):
    return review_repo.create(db, request)


def read_all(db: Session):
    return review_repo.get_all(db)


def read_one(db: Session, item_id: int):
    return review_repo.get_or_404(db, item_id)


def update(db: Session, item_id: int, request: ReviewsUpdate):
    return review_repo.update(db, item_id, request)


def delete(db: Session, item_id: int):
    review_repo.delete(db, item_id)