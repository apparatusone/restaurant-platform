from sqlalchemy.orm import Session
from shared.repositories import BaseRepository
from ..models.staff import Staff
from ..schemas.staff import StaffCreate, StaffUpdate
from ..security.hashing import hash_pin

# Initialize repository
staff_repo = BaseRepository[Staff, StaffCreate, StaffUpdate](Staff)


def create(db: Session, request: StaffCreate):
    """Create staff with hashed PIN"""
    # Hash the PIN before creating
    hashed = hash_pin(request.pin)
    
    # Create modified request with hashed PIN
    staff_data = request.model_dump(exclude={'pin'})
    staff_data['pin_hash'] = hashed
    
    # Create staff using repository
    new_staff = Staff(**staff_data)
    db.add(new_staff)
    db.commit()
    db.refresh(new_staff)
    return new_staff


def read_all(db: Session):
    return staff_repo.get_all(db)


def read_one(db: Session, id: int):
    return staff_repo.get_or_404(db, id)


def update(db: Session, id: int, request: StaffUpdate):
    """Update staff, preventing staff_id changes"""
    staff = staff_repo.get_or_404(db, id)
    
    update_data = request.model_dump(exclude_unset=True)
    
    # Prevent staff_id changes (security: prevents login code reassignment)
    update_data.pop("staff_id", None)
    
    for key, value in update_data.items():
        setattr(staff, key, value)
    
    db.commit()
    db.refresh(staff)
    return staff


def delete(db: Session, id: int):
    staff_repo.delete(db, id)
