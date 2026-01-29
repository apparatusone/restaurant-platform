from dotenv import load_dotenv
import os
from argon2 import PasswordHasher
from argon2.exceptions import VerifyMismatchError
from argon2.low_level import Type

load_dotenv()

ph = PasswordHasher(
    time_cost=2,
    memory_cost=65536,  # 64 MB
    parallelism=1,
    hash_len=32,
    salt_len=16,
    type=Type.ID
)

PEPPER = os.getenv("PIN_PEPPER")
if not PEPPER:
    raise RuntimeError("PIN_PEPPER environment variable is not set")

def hash_pin(pin: str) -> str:
    return ph.hash(pin + PEPPER)

def verify_pin(hash_value: str, pin: str) -> bool:
    try:
        ph.verify(hash_value, pin + PEPPER)
        return True
    except VerifyMismatchError:
        return False
