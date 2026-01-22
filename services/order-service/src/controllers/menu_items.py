from sqlalchemy.orm import Session
from shared.repositories import BaseRepository
from shared.models.menu_items import MenuItem
from shared.schemas.menu_items import MenuItemsCreate, MenuItemsUpdate

# Initialize repository
menu_item_repo = BaseRepository[MenuItem, MenuItemsCreate, MenuItemsUpdate](MenuItem)


def create(db: Session, request: MenuItemsCreate):
    return menu_item_repo.create(db, request)


def read_all(db: Session):
    return menu_item_repo.get_all(db)


def read_one(db: Session, item_id: int):
    return menu_item_repo.get_or_404(db, item_id)


def update(db: Session, item_id: int, request: MenuItemsUpdate):
    return menu_item_repo.update(db, item_id, request)


def delete(db: Session, item_id: int):
    menu_item_repo.delete(db, item_id)
    