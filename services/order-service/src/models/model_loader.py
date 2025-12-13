from . import menu_item_ingredients, resources, payment_method, promotions, reviews, customers, staff, tables, table_sessions, checks, robot_queue
from shared.models import orders, order_items, menu_items
from shared.dependencies.database import engine


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