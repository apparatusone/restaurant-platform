from . import orders, order_details, menu_items, menu_item_ingredients, resources, payment_method, promotions, reviews, customers

from ..dependencies.database import engine


def index():
    staff.Base.metadata.create_all(engine)  # Staff first (referenced by table_sessions)
    tables.Base.metadata.create_all(engine)  # Tables (references sessions)
    checks.Base.metadata.create_all(engine)  # Checks (references sessions)
    orders.Base.metadata.create_all(engine)
    order_details.Base.metadata.create_all(engine)
    resources.Base.metadata.create_all(engine)
    payment_method.Base.metadata.create_all(engine)
    promotions.Base.metadata.create_all(engine)
    reviews.Base.metadata.create_all(engine)
    customers.Base.metadata.create_all(engine)
    menu_items.Base.metadata.create_all(engine)
    menu_item_ingredients.Base.metadata.create_all(engine)