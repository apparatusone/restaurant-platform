from . import (
    check_items,
    recipes,
    menu_items,
    customers,
    checks
)

def load_routes(app):
    app.include_router(check_items.router)
    app.include_router(recipes.router)
    app.include_router(menu_items.router)
    app.include_router(customers.router)
    app.include_router(checks.router)
