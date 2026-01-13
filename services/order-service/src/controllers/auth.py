from fastapi import HTTPException, status
from sqlalchemy.orm import Session
from ..models.staff import Staff as StaffModel
from ..schemas.auth import PinLoginRequest, PinLoginResponse, AuthUser
from ..security.hashing import verify_pin
import time
from dotenv import load_dotenv
import os
import jwt

load_dotenv()

# FIX: user can continue to use a token even if their account is destroyed
# TODO: refresh tokens (stored server-side and hashed)

JWT_SECRET = os.getenv("JWT_SECRET")
if not JWT_SECRET:
    raise RuntimeError("JWT_SECRET environment variable is not set")
JWT_TTL_SECS = 12 * 60 * 60  # 12 hours

def _make_token(*, staff: StaffModel) -> str:
    now = int(time.time())
    payload = {
        "sub": str(staff.id),
        "sid": staff.staff_id,
        "name": staff.name,
        "role": staff.role,
        "iat": now,
        "exp": now + JWT_TTL_SECS,
    }
    token = jwt.encode(payload, JWT_SECRET, algorithm="HS256")
    if isinstance(token, bytes):
        token = token.decode("utf-8")
    return token

def validate_jwt_token(token: str) -> dict:
    """Validate JWT token and return result with payload or error details"""
    try:
        payload = jwt.decode(token, JWT_SECRET, algorithms=["HS256"])
        return {"valid": True, "payload": payload}
    except jwt.ExpiredSignatureError:
        return {"valid": False, "error": "expired", "message": "Token has expired"}
    except jwt.InvalidTokenError:
        return {"valid": False, "error": "invalid", "message": "Invalid token"}

def pin_login(db: Session, req: PinLoginRequest) -> PinLoginResponse:
    staff = db.query(StaffModel).filter(StaffModel.staff_id == req.staff_id, StaffModel.is_active == True).first()
    if not staff:
        raise HTTPException(status_code=status.HTTP_401_UNAUTHORIZED, detail="Wrong PIN")

    # check if account is locked out
    max_attempts = 3
    if (staff.failed_attempts or 0) >= max_attempts:
        raise HTTPException(status_code=status.HTTP_423_LOCKED, detail="Account locked due to too many failed attempts")

    if not verify_pin(staff.pin_hash, req.pin):
        # increment failed_attempts
        staff.failed_attempts = (staff.failed_attempts or 0) + 1
        db.commit()
        
        remaining_attempts = max_attempts - staff.failed_attempts
        if remaining_attempts <= 0:
            raise HTTPException(status_code=status.HTTP_423_LOCKED, detail="Account locked due to too many failed attempts")
        else:
            raise HTTPException(status_code=status.HTTP_401_UNAUTHORIZED, detail=f"Wrong PIN, {remaining_attempts} attempts remaining.")

    # TODO: track last login datetime
    staff.failed_attempts = 0
    db.commit()

    token = _make_token(staff=staff)
    return PinLoginResponse(
        token=token,
        user=AuthUser(id=staff.id, name=staff.name, role=staff.role, staff_id=staff.staff_id),
        expires_in=JWT_TTL_SECS,
    )