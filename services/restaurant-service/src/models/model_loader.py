from shared.dependencies.database import engine
from . import table, table_session, ingredient


def index():
    """Initialize restaurant-service database tables"""
    table.Base.metadata.create_all(engine)
    table_session.Base.metadata.create_all(engine)
    ingredient.Base.metadata.create_all(engine)
