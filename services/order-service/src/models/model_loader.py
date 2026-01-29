from shared.models import check_items, menu_items
from shared.dependencies.database import engine, Base

# Now import order-service specific models
from . import recipes, customers, checks


def index():
    """
    Initialize order-service database tables.

    """
    # Create all tables at once - Base.metadata has all registered models
    Base.metadata.create_all(engine, checkfirst=True)