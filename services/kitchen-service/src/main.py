import uvicorn
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from .routers import kitchen

app = FastAPI(
    title="Kitchen Service",
    description="""
    Kitchen management service for restaurant operations:
    
    * **Kitchen Queue** - Track order items in preparation
    * **Order Item Status** - pending → preparing → ready
    * **Kitchen Display System** - Real-time kitchen view
    * **Prep Timing** - Track preparation duration
    * **WebSocket Updates** - Real-time status updates
    """,
)

origins = ["*"]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include routers
app.include_router(kitchen.router)

@app.get("/")
def root():
    return {"service": "kitchen-service", "status": "running"}

@app.get("/health")
def health():
    return {"status": "healthy"}


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8002)
