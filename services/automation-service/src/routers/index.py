from . import automation, orders


def load_routes(app):
    app.include_router(automation.router)
    app.include_router(orders.router)
