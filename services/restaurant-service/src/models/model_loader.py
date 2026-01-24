from shared.dependencies.database import engine
from . import table, table_seating, ingredient


def index():
    """Initialize restaurant-service database tables"""
    table.Base.metadata.create_all(engine)
    table_seating.Base.metadata.create_all(engine)
    ingredient.Base.metadata.create_all(engine)
