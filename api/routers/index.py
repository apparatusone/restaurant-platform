from . import (
    orders,
    order_items,
    resources,
    menu_item_ingredients,
    menu_items,
    menu,
    payment_method,
    customers,
    administrator_actions,
    reviews,
    promotions,
    analytics,
    staff,
    auth,
    tables,
    table_sessions,
    checks
)

def load_routes(app):
    app.include_router(analytics.router)
    app.include_router(administrator_actions.router)
    app.include_router(orders.router)
    app.include_router(order_details.router)
    app.include_router(resources.router)
    app.include_router(menu_item_ingredients.router)
    app.include_router(menu_items.router)
    app.include_router(menu.router)
    app.include_router(payment_method.router)
    app.include_router(customers.router)
    app.include_router(reviews.router)
    app.include_router(promotions.router)
    app.include_router(staff.router)
    app.include_router(auth.router)
    app.include_router(tables.router)
    app.include_router(table_sessions.router)
    app.include_router(checks.router)
