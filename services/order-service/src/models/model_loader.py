from shared.models import check_items, menu_items
from shared.dependencies.database import engine
from sqlalchemy.orm import relationship

# Add relationships that reference order-service-specific models BEFORE importing other models
# These are not in shared to allow kitchen-service to use the models without these dependencies
if not hasattr(menu_items.MenuItem, 'recipes'):
    menu_items.MenuItem.recipes = relationship("Recipe", back_populates="menu_item", cascade="all, delete-orphan")
if not hasattr(menu_items.MenuItem, 'reviews'):
    menu_items.MenuItem.reviews = relationship("Reviews", back_populates="menu_item", cascade="all, delete-orphan")

# Now import order-service specific models
from . import recipes, ingredients, payment_method, promotions, reviews, customers, staff, tables, table_sessions, checks, robot_queue


def index():
    
    # Create tables in dependency order
    staff.Base.metadata.create_all(engine)  # Staff first (referenced by table_sessions)
    table_sessions.Base.metadata.create_all(engine)  # Sessions next (referenced by tables)
    tables.Base.metadata.create_all(engine)  # Tables (references sessions)
    checks.Base.metadata.create_all(engine)  # Checks (references sessions)
    
    check_items.Base.metadata.create_all(engine)  # CheckItems (references checks)
    robot_queue.Base.metadata.create_all(engine)  # Robot queue
    ingredients.Base.metadata.create_all(engine)  # Ingredients
    payment_method.Base.metadata.create_all(engine)
    promotions.Base.metadata.create_all(engine)
    reviews.Base.metadata.create_all(engine)
    customers.Base.metadata.create_all(engine)
    menu_items.Base.metadata.create_all(engine)
    recipes.Base.metadata.create_all(engine)  # Recipes (references menu_items and ingredients)