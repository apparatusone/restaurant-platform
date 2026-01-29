from shared.dependencies.database import engine
from shared.dependencies.database import Base
from . import staff
from . import timeclock


def index():
    """Initialize staff-service database tables"""
    Base.metadata.create_all(engine)
