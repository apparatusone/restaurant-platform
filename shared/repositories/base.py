"""
Base repository pattern for standardized CRUD operations.
"""
from typing import Generic, TypeVar, Type, Optional, List, Any, Dict
from sqlalchemy.orm import Session
from sqlalchemy.exc import SQLAlchemyError, IntegrityError
from fastapi import HTTPException, status
from pydantic import BaseModel
import logging

logger = logging.getLogger(__name__)

ModelType = TypeVar("ModelType")
CreateSchemaType = TypeVar("CreateSchemaType", bound=BaseModel)
UpdateSchemaType = TypeVar("UpdateSchemaType", bound=BaseModel)


class BaseRepository(Generic[ModelType, CreateSchemaType, UpdateSchemaType]):
    """
    Base repository providing standard CRUD operations.
    
    Usage:
        from shared.repositories import BaseRepository
        from .models.customer import Customer
        from .schemas.customer import CustomerCreate, CustomerUpdate
        
        customer_repo = BaseRepository[Customer, CustomerCreate, CustomerUpdate](Customer)
        
        # In controller
        def read_one(db: Session, id: int):
            return customer_repo.get_or_404(db, id)
    """
    
    def __init__(self, model: Type[ModelType]):
        self.model = model
        self.model_name = model.__name__
    
    def get(self, db: Session, id: Any) -> Optional[ModelType]:
        """Get a single record by ID, returns None if not found."""
        try:
            return db.query(self.model).filter(self.model.id == id).first()
        except SQLAlchemyError as e:
            logger.error(
                f"Database error fetching {self.model_name}",
                extra={"model": self.model_name, "id": id, "operation": "get"},
                exc_info=True
            )
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail=f"Database error while fetching {self.model_name}"
            )
    
    def get_or_404(self, db: Session, id: Any) -> ModelType:
        """Get a single record by ID, raises 404 if not found."""
        obj = self.get(db, id)
        if not obj:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"{self.model_name} with id {id} not found"
            )
        return obj
    
    def get_all(self, db: Session, skip: int = 0, limit: int = 1000) -> List[ModelType]:
        """Get all records with optional pagination."""
        try:
            return db.query(self.model).offset(skip).limit(limit).all()
        except SQLAlchemyError as e:
            logger.error(
                f"Database error fetching {self.model_name} list",
                extra={"model": self.model_name, "operation": "get_all", "skip": skip, "limit": limit},
                exc_info=True
            )
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail=f"Database error while fetching {self.model_name} list"
            )
    
    def get_by_field(self, db: Session, field_name: str, value: Any) -> Optional[ModelType]:
        """Get a single record by any field."""
        try:
            field = getattr(self.model, field_name)
            return db.query(self.model).filter(field == value).first()
        except AttributeError:
            raise ValueError(f"{self.model_name} has no field '{field_name}'")
        except SQLAlchemyError as e:
            logger.error(
                f"Database error fetching {self.model_name} by field",
                extra={"model": self.model_name, "field": field_name, "value": value, "operation": "get_by_field"},
                exc_info=True
            )
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail=f"Database error while fetching {self.model_name}"
            )
    
    def get_by_field_or_404(self, db: Session, field_name: str, value: Any) -> ModelType:
        """Get a single record by any field, raises 404 if not found."""
        obj = self.get_by_field(db, field_name, value)
        if not obj:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"{self.model_name} with {field_name}={value} not found"
            )
        return obj
    
    def filter_by(self, db: Session, **filters) -> List[ModelType]:
        """Get records matching filter criteria."""
        try:
            query = db.query(self.model)
            for field_name, value in filters.items():
                if hasattr(self.model, field_name):
                    field = getattr(self.model, field_name)
                    query = query.filter(field == value)
            return query.all()
        except SQLAlchemyError as e:
            logger.error(
                f"Database error filtering {self.model_name}",
                extra={"model": self.model_name, "filters": filters, "operation": "filter_by"},
                exc_info=True
            )
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail=f"Database error while filtering {self.model_name}"
            )
    
    def create(self, db: Session, obj_in: CreateSchemaType) -> ModelType:
        """Create a new record."""
        try:
            # Convert Pydantic model to dict
            if hasattr(obj_in, 'model_dump'):
                obj_data = obj_in.model_dump()
            else:
                obj_data = obj_in.dict()
            
            db_obj = self.model(**obj_data)
            db.add(db_obj)
            db.commit()
            db.refresh(db_obj)
            logger.info(f"Created {self.model_name}", extra={"model": self.model_name, "id": db_obj.id, "operation": "create"})
            return db_obj
        except IntegrityError as e:
            db.rollback()
            # Check for common constraint violations
            error_msg = str(e.orig) if hasattr(e, 'orig') else str(e)
            logger.warning(
                f"Integrity error creating {self.model_name}: {error_msg}",
                extra={"model": self.model_name, "operation": "create", "data": obj_data},
                exc_info=True
            )
            if 'UNIQUE constraint' in error_msg or 'Duplicate entry' in error_msg:
                raise HTTPException(
                    status_code=status.HTTP_409_CONFLICT,
                    detail=f"{self.model_name} already exists"
                )
            elif 'FOREIGN KEY constraint' in error_msg or 'foreign key' in error_msg.lower():
                raise HTTPException(
                    status_code=status.HTTP_400_BAD_REQUEST,
                    detail="Referenced resource does not exist"
                )
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail=f"Database constraint violation: {error_msg}"
            )
        except SQLAlchemyError as e:
            db.rollback()
            logger.error(
                f"Database error creating {self.model_name}",
                extra={"model": self.model_name, "operation": "create"},
                exc_info=True
            )
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail=f"Database error while creating {self.model_name}"
            )
    
    def update(self, db: Session, id: Any, obj_in: UpdateSchemaType) -> ModelType:
        """Update an existing record."""
        db_obj = self.get_or_404(db, id)
        
        try:
            # Convert Pydantic model to dict, excluding unset fields
            if hasattr(obj_in, 'model_dump'):
                update_data = obj_in.model_dump(exclude_unset=True)
            else:
                update_data = obj_in.dict(exclude_unset=True)
            
            for field, value in update_data.items():
                setattr(db_obj, field, value)
            
            db.commit()
            db.refresh(db_obj)
            logger.info(f"Updated {self.model_name}", extra={"model": self.model_name, "id": id, "operation": "update"})
            return db_obj
        except IntegrityError as e:
            db.rollback()
            error_msg = str(e.orig) if hasattr(e, 'orig') else str(e)
            logger.warning(
                f"Integrity error updating {self.model_name}: {error_msg}",
                extra={"model": self.model_name, "id": id, "operation": "update"},
                exc_info=True
            )
            if 'UNIQUE constraint' in error_msg or 'Duplicate entry' in error_msg:
                raise HTTPException(
                    status_code=status.HTTP_409_CONFLICT,
                    detail=f"{self.model_name} with this value already exists"
                )
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail=f"Database constraint violation: {error_msg}"
            )
        except SQLAlchemyError as e:
            db.rollback()
            logger.error(
                f"Database error updating {self.model_name}",
                extra={"model": self.model_name, "id": id, "operation": "update"},
                exc_info=True
            )
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail=f"Database error while updating {self.model_name}"
            )
    
    def delete(self, db: Session, id: Any) -> None:
        """Delete a record by ID."""
        db_obj = self.get_or_404(db, id)
        
        try:
            db.delete(db_obj)
            db.commit()
            logger.info(f"Deleted {self.model_name}", extra={"model": self.model_name, "id": id, "operation": "delete"})
        except IntegrityError as e:
            db.rollback()
            logger.warning(
                f"Cannot delete {self.model_name}: referenced by other records",
                extra={"model": self.model_name, "id": id, "operation": "delete"},
                exc_info=True
            )
            raise HTTPException(
                status_code=status.HTTP_409_CONFLICT,
                detail=f"Cannot delete {self.model_name}: it is referenced by other records"
            )
        except SQLAlchemyError as e:
            db.rollback()
            logger.error(
                f"Database error deleting {self.model_name}",
                extra={"model": self.model_name, "id": id, "operation": "delete"},
                exc_info=True
            )
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail=f"Database error while deleting {self.model_name}"
            )
    
    def exists(self, db: Session, id: Any) -> bool:
        """Check if a record exists by ID."""
        return self.get(db, id) is not None
    
    def count(self, db: Session, **filters) -> int:
        """Count records matching filter criteria."""
        try:
            query = db.query(self.model)
            for field_name, value in filters.items():
                if hasattr(self.model, field_name):
                    field = getattr(self.model, field_name)
                    query = query.filter(field == value)
            return query.count()
        except SQLAlchemyError as e:
            logger.error(
                f"Database error counting {self.model_name}",
                extra={"model": self.model_name, "filters": filters, "operation": "count"},
                exc_info=True
            )
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail=f"Database error while counting {self.model_name}"
            )
