from shared.dependencies.database import engine
from . import staff


def index():
    """Initialize staff-service database tables"""
    staff.Base.metadata.create_all(engine)
