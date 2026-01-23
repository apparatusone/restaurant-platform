"""
Restaurant Service - Main Application

Manages tables, table sessions, ingredients, and kitchen operations.
"""

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from .routers import table, table_session, ingredient, kitchen

app = FastAPI(
    title="Restaurant Service",
    description="Manages tables, table sessions, ingredients, and kitchen operations",
    version="1.0.0"
)

# CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Register routers
app.include_router(table.router)
app.include_router(table_session.router)
app.include_router(ingredient.router)
app.include_router(kitchen.router)


@app.get("/")
async def root():
    """Health check endpoint"""
    return {
        "service": "restaurant-service",
        "status": "running",
        "version": "1.0.0"
    }


@app.get("/health")
async def health():
    """Health check endpoint"""
    return {"status": "healthy"}


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8003)
