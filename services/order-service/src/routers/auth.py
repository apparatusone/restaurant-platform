# api/routers/auth.py
from fastapi import APIRouter, Depends
from sqlalchemy.orm import Session
from shared.dependencies.database import get_db
from ..schemas.auth import PinLoginRequest, PinLoginResponse
from ..controllers.auth import pin_login

router = APIRouter(prefix="/auth", tags=["auth"])

@router.post("/pin", response_model=PinLoginResponse, status_code=200)
def login_with_pin(request: PinLoginRequest, db: Session = Depends(get_db)):
    return pin_login(db, request)