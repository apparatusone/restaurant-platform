from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from .routers import payment

app = FastAPI(
    title="Payment Service",
    description="Service for managing payment processing, receipts, and payment history",
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
app.include_router(payment.router)

@app.get("/")
async def root():
    """Root endpoint"""
    return {"service": "payment-service", "status": "running"}


@app.get("/health")
async def health_check():
    return {"status": "healthy", "service": "payment-service"}


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8004)
