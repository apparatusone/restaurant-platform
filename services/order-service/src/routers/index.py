from . import (
    check_items,
    ingredients,
    recipes,
    menu_items,
    customers,
    administrator_actions,
    promotions,
    staff,
    auth,
    config,
    tables,
    table_sessions,
    checks,
    robot
)

def load_routes(app):
    app.include_router(administrator_actions.router)
    app.include_router(check_items.router)
    app.include_router(ingredients.router)
    app.include_router(recipes.router)
    app.include_router(menu_items.router)
    app.include_router(customers.router)
    app.include_router(promotions.router)
    app.include_router(staff.router)
    app.include_router(auth.router)
    app.include_router(config.router)
    app.include_router(tables.router)
    app.include_router(table_sessions.router)
    app.include_router(checks.router)
    app.include_router(robot.router)
