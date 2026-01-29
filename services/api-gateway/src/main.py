import logging
from fastapi import FastAPI, Request, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
import httpx
import sys
from pathlib import Path
from dotenv import load_dotenv
import os
import jwt

load_dotenv()
ENV = os.getenv("ENV", "local")

# Add project root to path for shared imports
sys.path.insert(0, str(Path(__file__).parent.parent.parent))
from shared.utils.http_client import ResilientHttpClient, CircuitBreakerOpenError

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

JWT_SECRET = os.getenv("JWT_SECRET")
if not JWT_SECRET:
    raise RuntimeError("JWT_SECRET environment variable is not set")

app = FastAPI(
    title="API Gateway",
    description="Central entry point for restaurant services",
    version="1.0.0"
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


# =============================================================================
# JWT VALIDATION MIDDLEWARE
# =============================================================================

@app.middleware("http")
async def jwt_validation_middleware(request: Request, call_next):
    """Validate JWT tokens for protected endpoints"""
    
    # Skip validation for OPTIONS requests (CORS preflight)
    if request.method == "OPTIONS":
        return await call_next(request)
    
    # Skip validation for unprotected paths
    if not is_protected_path(request.url.path):
        return await call_next(request)
    
    # Extract token from Authorization header
    authorization = request.headers.get("authorization")
    
    if not authorization:
        return JSONResponse(
            status_code=401,
            content={"detail": "Missing Authorization header"}
        )
    
    try:
        token = extract_token_from_header(authorization)
    except HTTPException as e:
        return JSONResponse(
            status_code=e.status_code,
            content={"detail": e.detail}
        )
    
    validation_result = validate_jwt_token(token)
    
    if not validation_result["valid"]:
        return JSONResponse(
            status_code=401,
            content={"detail": validation_result.get("message", "Invalid token")}
        )
    
    # Attach staff info to request state for downstream services
    payload = validation_result["payload"]
    request.state.staff_id = payload.get("sub")
    request.state.staff_sid = payload.get("sid")
    request.state.staff_name = payload.get("name")
    request.state.staff_role = payload.get("role")

    if request.url.path.startswith("/timeclock"):
        return await call_next(request)

    try:
        staff_client = SERVICE_CLIENTS["staff-service"]
        res = await staff_client.get(
            "timeclock/status",
            headers={"X-Staff-Id": str(request.state.staff_id)}
        )
        status_payload = res.json()
        if not status_payload.get("clocked_in"):
            return JSONResponse(
                status_code=403,
                content={"detail": "User must be clocked in"}
            )
    except CircuitBreakerOpenError:
        return JSONResponse(
            status_code=503,
            content={"detail": "Staff service unavailable"}
        )
    except Exception:
        return JSONResponse(
            status_code=503,
            content={"detail": "Unable to verify clock-in status"}
        )
    
    return await call_next(request)

# =============================================================================
# SERVICE CONFIGURATION
# =============================================================================

# maps service names to their base URLs
SERVICE_URLS = {
    "staff-service": os.getenv("STAFF_SERVICE_URL", "http://localhost:8001"),
    "order-service": os.getenv("ORDER_SERVICE_URL", "http://localhost:8002"),
    "restaurant-service": os.getenv("RESTAURANT_SERVICE_URL", "http://localhost:8003"),
    "payment-service": os.getenv("PAYMENT_SERVICE_URL", "http://localhost:8004"),
    "automation-service": os.getenv("AUTOMATION_SERVICE_URL", "http://localhost:8005"),
}

# Initialize resilient HTTP clients for each service
SERVICE_CLIENTS = {
    name: ResilientHttpClient(
        base_url=url,
        max_retries=3,
        timeout=5.0
    )
    for name, url in SERVICE_URLS.items()
}

# maps URL path prefixes to service names
ROUTE_MAP = {
    "auth": "staff-service",
    "staff": "staff-service",
    "timeclock": "staff-service",
    "roles": "staff-service",
    "permissions": "staff-service",
    
    "menu-items": "order-service",
    "recipes": "order-service",
    "checks": "order-service",
    "check-items": "order-service",
    
    "tables": "restaurant-service",
    "seatings": "restaurant-service",
    "ingredients": "restaurant-service",
    "kitchen": "restaurant-service",
    
    "payments": "payment-service",
    "receipts": "payment-service",
    
    "automation": "automation-service",
    "robots": "automation-service",
    "tasks": "automation-service"
}

# Don't require JWT validation
UNPROTECTED_PATHS = [
    "/auth/pin",
    "/auth/login",
    "/health",
    "/",
]

if ENV != "prod":
    UNPROTECTED_PATHS.extend([
        "/docs",
        "/openapi.json",
    ])

# =============================================================================
# HELPERS
# =============================================================================

def validate_jwt_token(token: str) -> dict:
    """Validate JWT token and return result with payload or error details
    
    Copied from services/staff-service/src/controllers/auth.py
    """
    try:
        payload = jwt.decode(token, JWT_SECRET, algorithms=["HS256"])
        return {"valid": True, "payload": payload}
    except jwt.ExpiredSignatureError:
        return {"valid": False, "error": "expired", "message": "Token has expired"}
    except jwt.InvalidTokenError:
        return {"valid": False, "error": "invalid", "message": "Invalid token"}


def extract_token_from_header(authorization: str) -> str:
    """Extract JWT token from Authorization header"""
    if not authorization:
        raise HTTPException(status_code=401, detail="Missing Authorization header")
    
    parts = authorization.split()
    if len(parts) != 2 or parts[0].lower() != "bearer":
        raise HTTPException(status_code=401, detail="Invalid Authorization header format. Expected: Bearer <token>")
    
    return parts[1]


def is_protected_path(path: str) -> bool:
    """Check if path requires JWT validation"""
    for unprotected in UNPROTECTED_PATHS:
        if path == unprotected:
            return False
        # Check if path starts with unprotected path followed by /
        if path.startswith(unprotected + "/"):
            return False
    return True


def get_service_for_resource(resource: str) -> str:
    """Determine which service handles this resource"""
    if resource in ROUTE_MAP:
        return ROUTE_MAP[resource]
    
    raise HTTPException(status_code=404, detail=f"No service configured for resource '{resource}'")


def normalize_path(path: str) -> str:
    """Normalize path to avoid FastAPI redirect issues.
    
    Only add trailing slash to resource roots (e.g., /tables, /orders).
    Don't add to sub-paths (e.g., /table-seatings/active).
    """
    # Count path segments (excluding empty strings from leading/trailing slashes)
    segments = [s for s in path.split('/') if s]
    
    # Only add trailing slash for single-segment paths (resource roots)
    # e.g., /tables -> /tables/  but /tables/1 stays as-is
    if len(segments) == 1 and not path.endswith('/'):
        return path + '/'
    
    return path


# =============================================================================
# PROXY
# =============================================================================

async def proxy_request(service_name: str, path: str, request: Request):
    """Proxy request to backend service with error handling"""
    if service_name not in SERVICE_CLIENTS:
        raise HTTPException(status_code=404, detail=f"Service '{service_name}' not found")
    
    client = SERVICE_CLIENTS[service_name]
    path = normalize_path(path)
    
    try:
        body = await request.body() if request.method in ["POST", "PUT", "PATCH"] else None
        
        logger.info(f"{request.method} {client.base_url}{path}")
        
        # Build headers to forward to downstream service
        headers = {"content-type": request.headers.get("content-type", "application/json")}
        
        # Attach staff info from request state if available
        if hasattr(request.state, "staff_id"):
            headers["X-Staff-Id"] = str(request.state.staff_id)
        if hasattr(request.state, "staff_sid"):
            headers["X-Staff-SID"] = str(request.state.staff_sid)
        if hasattr(request.state, "staff_name"):
            headers["X-Staff-Name"] = str(request.state.staff_name)
        if hasattr(request.state, "staff_role"):
            headers["X-Staff-Role"] = str(request.state.staff_role)
        
        # Use resilient client with retry logic and circuit breaker
        response = await client.request(
            method=request.method,
            path=path,
            content=body,
            params=dict(request.query_params),
            headers=headers,
        )
        
        logger.info(f"{request.method} {client.base_url}{path} -> {response.status_code}")
        
        # Parse response
        try:
            content = response.json() if response.text else {}
        except Exception:
            content = {"detail": response.text}
        
        return JSONResponse(status_code=response.status_code, content=content)
    
    except CircuitBreakerOpenError as e:
        logger.error(f"Circuit breaker open for {service_name}: {e}")
        raise HTTPException(status_code=503, detail=f"Service '{service_name}' unavailable (circuit breaker open)")
    except httpx.ConnectError:
        logger.error(f"Cannot connect to {service_name}")
        raise HTTPException(status_code=503, detail=f"Service '{service_name}' unavailable")
    except httpx.TimeoutException:
        logger.error(f"Timeout connecting to {service_name}")
        raise HTTPException(status_code=504, detail=f"Service '{service_name}' timeout")
    except httpx.HTTPStatusError as e:
        # Pass through HTTP errors from backend services
        logger.warning(f"Backend service error: {e.response.status_code}")
        try:
            content = e.response.json()
        except Exception:
            content = {"detail": e.response.text}
        return JSONResponse(status_code=e.response.status_code, content=content)
    except Exception as e:
        logger.error(f"Proxy error: {e}", exc_info=True)
        raise HTTPException(status_code=502, detail="Bad gateway")


# =============================================================================
# ROUTES
# =============================================================================

@app.get("/health")
async def health_check():
    """Check gateway and service health"""
    health = {"gateway": "healthy", "services": {}}
    
    for name, client in SERVICE_CLIENTS.items():
        try:
            response = await client.get("/health")
            health["services"][name] = "healthy" if response.status_code == 200 else "unhealthy"
        except Exception:
            health["services"][name] = "unreachable"
    
    return health

# expose /docs if local
@app.get("/")
async def root():
    payload = {
        "service": "api-gateway",
        "version": "1.0.0",
    }
    if ENV != "prod":
        payload["docs"] = "/docs"
    return payload


@app.api_route("/{resource}", methods=["GET", "POST", "PUT", "PATCH", "DELETE"])
async def proxy_resource_root(resource: str, request: Request):
    service_name = get_service_for_resource(resource)
    return await proxy_request(service_name, f"/{resource}", request)


@app.api_route("/{resource}/{path:path}", methods=["GET", "POST", "PUT", "PATCH", "DELETE"])
async def proxy_resource_path(resource: str, path: str, request: Request):
    service_name = get_service_for_resource(resource)
    return await proxy_request(service_name, f"/{resource}/{path}", request)


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
