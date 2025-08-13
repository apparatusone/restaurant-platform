from pydantic import BaseModel, Field

class PinLoginRequest(BaseModel):
    staff_id: str = Field(..., min_length=4, max_length=4)
    pin: str = Field(..., min_length=4, max_length=6)

class AuthUser(BaseModel):
    id: int
    name: str
    role: str
    staff_id: str

class PinLoginResponse(BaseModel):
    token: str
    user: AuthUser
    expires_in: int # seconds