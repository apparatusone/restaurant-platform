from sqlalchemy.orm import Session
from shared.repositories import BaseRepository
from ..models.resources import Resource
from ..schemas.resources import ResourceCreate, ResourceUpdate

# Initialize repository
resource_repo = BaseRepository[Resource, ResourceCreate, ResourceUpdate](Resource)


def create(db: Session, request: ResourceCreate):
    return resource_repo.create(db, request)


def read_all(db: Session):
    return resource_repo.get_all(db)


def read_one(db: Session, item_id: int):
    return resource_repo.get_or_404(db, item_id)


def update(db: Session, item_id: int, request: ResourceUpdate):
    return resource_repo.update(db, item_id, request)


def delete(db: Session, item_id: int):
    resource_repo.delete(db, item_id)