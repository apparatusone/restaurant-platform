"""
HTTP clients for inter-service communication

This module provides HTTP clients for calling other microservices in the architecture.
Following the shared database pattern:
- Services can READ from any table directly via database
- Services MUST call owner service for WRITE operations

Usage:
    from ..clients import restaurant_client, payment_client, staff_client
    
    # Call restaurant-service for write operations (e.g., kitchen queue)
    response = await restaurant_client.post("/kitchen/queue", json={...})
    
    # Call payment-service for payment creation (write operation)
    response = await payment_client.post("/payments", json={...})
    
    # Call staff-service for staff validation (if needed)
    response = await staff_client.get(f"/staff/{staff_id}")
"""
import os
from shared.utils.http_client import ResilientHttpClient

# Initialize HTTP clients for other services
restaurant_client = ResilientHttpClient(
    base_url=os.getenv("RESTAURANT_SERVICE_URL", "http://localhost:8003"),
    max_retries=3,
    timeout=5.0
)

payment_client = ResilientHttpClient(
    base_url=os.getenv("PAYMENT_SERVICE_URL", "http://localhost:8004"),
    max_retries=3,
    timeout=5.0
)

staff_client = ResilientHttpClient(
    base_url=os.getenv("STAFF_SERVICE_URL", "http://localhost:8001"),
    max_retries=3,
    timeout=5.0
)
