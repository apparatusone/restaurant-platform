from shared.models import orders, order_items, menu_items
from shared.dependencies.database import engine
from sqlalchemy.orm import relationship

# Add relationships that reference order-service-specific models BEFORE importing other models
# These are not in shared to allow kitchen-service to use the models without these dependencies
if not hasattr(orders.Order, 'check'):
    orders.Order.check = relationship("Check", back_populates="order")
    orders.Order.customer = relationship("Customer", back_populates="orders")
    orders.Order.promo = relationship("Promotion", back_populates="orders")
    orders.Order.payments = relationship("Payment", back_populates="order")

if not hasattr(menu_items.MenuItem, 'menu_item_ingredients'):
    menu_items.MenuItem.menu_item_ingredients = relationship("MenuItemIngredient", back_populates="menu_item", cascade="all, delete-orphan")
    menu_items.MenuItem.reviews = relationship("Reviews", back_populates="menu_item", cascade="all, delete-orphan")

# Now import order-service specific models
from . import menu_item_ingredients, resources, payment_method, promotions, reviews, customers, staff, tables, table_sessions, checks, robot_queue


def index():
    
    # Create tables in dependency order
    staff.Base.metadata.create_all(engine)  # Staff first (referenced by table_sessions)
    table_sessions.Base.metadata.create_all(engine)  # Sessions next (referenced by tables)
    tables.Base.metadata.create_all(engine)  # Tables (references sessions)
    checks.Base.metadata.create_all(engine)  # Checks (references sessions)
    
    orders.Base.metadata.create_all(engine)
    order_items.Base.metadata.create_all(engine)
    robot_queue.Base.metadata.create_all(engine)  # Robot queue (references orders)
    resources.Base.metadata.create_all(engine)
    payment_method.Base.metadata.create_all(engine)
    promotions.Base.metadata.create_all(engine)
    reviews.Base.metadata.create_all(engine)
    customers.Base.metadata.create_all(engine)
    menu_items.Base.metadata.create_all(engine)
    menu_item_ingredients.Base.metadata.create_all(engine)