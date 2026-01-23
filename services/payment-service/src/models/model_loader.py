from shared.dependencies.database import engine
from . import payment


def index():
    """Initialize payment-service database tables"""
    payment.Base.metadata.create_all(engine)
