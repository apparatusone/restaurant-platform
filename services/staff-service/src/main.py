"""
Staff Service - Main Application

Manages staff accounts, authentication, and authorization.
"""

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from .models import model_loader
from .routers import staff, auth

app = FastAPI(
    title="Staff Service",
    description="Manages staff accounts, authentication, and authorization",
    version="1.0.0"
)

# Initialize database tables
model_loader.index()

# CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Register routers
app.include_router(auth.router)
app.include_router(staff.router)


@app.get("/")
async def root():
    """Health check endpoint"""
    return {"service": "staff-service", "status": "running"}


@app.get("/health")
async def health():
    """Health check endpoint"""
    return {"status": "healthy"}


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8001)
