import uvicorn
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
import threading

from .models import model_loader
from .routers import index as indexRoute
from .services.kitchen_subscriber import start_subscriber
from shared.utils.error_handlers import create_global_exception_handler
from shared.utils.logging_config import setup_logging

setup_logging("automation-service", level="INFO")

app = FastAPI(
    title="Automation Service",
    description="Robot automation and control service",
    version="1.0.0",
)

origins = ["*"]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

handler = create_global_exception_handler("automation-service")
app.add_exception_handler(Exception, handler)

model_loader.index()
indexRoute.load_routes(app)


@app.on_event("startup")
async def startup_event():
    # Start Redis subscriber in background thread
    subscriber_thread = threading.Thread(target=start_subscriber, daemon=True)
    subscriber_thread.start()


@app.on_event("shutdown")
async def shutdown_event():
    model_loader.shutdown()


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8005)
