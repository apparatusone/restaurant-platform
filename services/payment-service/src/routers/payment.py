from fastapi import APIRouter, Depends
from sqlalchemy.orm import Session
from ..controllers import payment as controller
from ..schemas import payment as schema
from shared.dependencies.database import get_db

router = APIRouter(
    tags=['Payment Method'],
    prefix="/payment_method",
)


@router.post("/", response_model=schema.Payment)
async def create(request: schema.PaymentCreate, db: Session = Depends(get_db)):
    return await controller.create(db=db, request=request)


@router.get("/", response_model=list[schema.Payment])
def read_all(db: Session = Depends(get_db)):
    return controller.read_all(db)


@router.get("/{item_id}", response_model=schema.Payment)
def read_one(item_id: int, db: Session = Depends(get_db)):
    return controller.read_one(db, item_id=item_id)

@router.put("/{item_id}", response_model=schema.Payment)
def update(item_id: int, request: schema.PaymentUpdate, db: Session = Depends(get_db)):
    return controller.update(db=db, request=request, item_id=item_id)


@router.delete("/{item_id}")
def delete(item_id: int, db: Session = Depends(get_db)):
    return controller.delete(db=db, item_id=item_id)


@router.get("/check/{check_id}", response_model=list[schema.Payment])
def get_payments_by_check(check_id: int, db: Session = Depends(get_db)):
    """Get all payments for a specific check"""
    return controller.read_by_check(db=db, check_id=check_id)


@router.get("/check/{check_id}/summary")
def get_check_payment_summary(check_id: int, db: Session = Depends(get_db)):
    """Get payment summary for a check including balance information"""
    return controller.get_check_payment_summary(db=db, check_id=check_id)
