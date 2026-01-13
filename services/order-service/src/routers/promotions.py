from fastapi import APIRouter, Depends, Query
from sqlalchemy.orm import Session
from typing import Optional
from ..controllers import promotions as controller
from ..schemas import promotions as schema
from ..services import staff as staff_services
from shared.dependencies.database import get_db

router = APIRouter(
    tags=['Promotions'],
    prefix="/promotions"
)


@router.get("/", response_model=list[schema.Promotion])
def get_promotions(
    code: Optional[str] = Query(None, description="Filter by promotion code"),
    active_only: Optional[bool] = Query(None, description="Filter active promotions only"),
    discount_type: Optional[str] = Query(None, description="Filter by discount type"),
    db: Session = Depends(get_db)
):
    """
    Get promotions with optional filtering by code, active status, or discount type
    """
    if code:
        return [staff_services.get_promotion_by_code(db, promo_code=code)]
    
    promotions = controller.read_all(db)
    
    # apply filters
    if active_only is not None:
        # filter based on active status - this would need to be implemented in the service
        pass
    
    if discount_type:
        # filter based on discount type - this would need to be implemented in the service
        pass
    
    return promotions


@router.get("/{promo_code}", response_model=schema.Promotion)
def get_promotion_by_code(promo_code: str, db: Session = Depends(get_db)):
    """
    Get promotion by code
    """
    return staff_services.get_promotion_by_code(db, promo_code=promo_code)


@router.put("/{promo_code}", response_model=schema.Promotion)
def update_promotion_by_code(promo_code: str, request: schema.PromotionUpdate, db: Session = Depends(get_db)):
    """
    Update promotion by code
    """
    return staff_services.update_promotion_by_code(db=db, promo_code=promo_code, request=request)


@router.delete("/{promo_code}")
def delete_promotion_by_code(promo_code: str, db: Session = Depends(get_db)):
    """
    Delete promotion by code
    """
    return staff_services.delete_promotion_by_code(db=db, promo_code=promo_code)


# admin endpoints for promotion management
@router.post("/", response_model=schema.Promotion)
def create(request: schema.PromotionCreate, db: Session = Depends(get_db)):
    return controller.create(db=db, request=request)


@router.get("/admin/all", response_model=list[schema.Promotion])
def read_all(db: Session = Depends(get_db)):
    return controller.read_all(db)


@router.get("/admin/{item_id}", response_model=schema.Promotion)
def read_one(item_id: int, db: Session = Depends(get_db)):
    return controller.read_one(db, item_id=item_id)


@router.put("/admin/{item_id}", response_model=schema.Promotion)
def update(item_id: int, request: schema.PromotionUpdate, db: Session = Depends(get_db)):
    return controller.update(db=db, request=request, item_id=item_id)


@router.delete("/admin/{item_id}")
def delete(item_id: int, db: Session = Depends(get_db)):
    return controller.delete(db=db, item_id=item_id)