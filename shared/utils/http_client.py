"""
HTTP Client with Retry Logic and Circuit Breaker Pattern

Provides resilient HTTP communication between microservices with:
- Exponential backoff for transient failures
- Configurable max retry attempts (default: 3)
- Configurable timeout (default: 5 seconds)
- Circuit breaker pattern to prevent cascading failures
"""

import httpx
import asyncio
import logging
from typing import Optional, Dict, Any
from datetime import datetime, timedelta
from enum import Enum

logger = logging.getLogger(__name__)


class CircuitState(Enum):
    """Circuit breaker states"""
    CLOSED = "closed"  # Normal operation
    OPEN = "open"  # Failing, reject requests
    HALF_OPEN = "half_open"  # Testing if service recovered


class CircuitBreaker:
    """Circuit breaker to prevent cascading failures"""
    
    def __init__(
        self,
        failure_threshold: int = 5,
        recovery_timeout: int = 30,
        success_threshold: int = 1
    ):
        """
        Initialize circuit breaker
        
        Args:
            failure_threshold: Number of consecutive failures before opening circuit
            recovery_timeout: Seconds to wait before attempting recovery (half-open)
            success_threshold: Number of successes in half-open state to close circuit
        """
        self.failure_threshold = failure_threshold
        self.recovery_timeout = recovery_timeout
        self.success_threshold = success_threshold
        
        self.state = CircuitState.CLOSED
        self.failure_count = 0
        self.success_count = 0
        self.last_failure_time: Optional[datetime] = None
    
    def can_attempt(self) -> bool:
        """Check if request can be attempted"""
        if self.state == CircuitState.CLOSED:
            return True
        
        if self.state == CircuitState.OPEN:
            # Check if recovery timeout has elapsed
            if self.last_failure_time:
                elapsed = (datetime.now() - self.last_failure_time).total_seconds()
                if elapsed >= self.recovery_timeout:
                    logger.info("Circuit breaker entering half-open state")
                    self.state = CircuitState.HALF_OPEN
                    self.success_count = 0
                    return True
            return False
        
        # HALF_OPEN state - allow attempt
        return True
    
    def record_success(self):
        """Record successful request"""
        if self.state == CircuitState.HALF_OPEN:
            self.success_count += 1
            if self.success_count >= self.success_threshold:
                logger.info("Circuit breaker closing after successful recovery")
                self.state = CircuitState.CLOSED
                self.failure_count = 0
        elif self.state == CircuitState.CLOSED:
            # Reset failure count on success
            self.failure_count = 0
    
    def record_failure(self):
        """Record failed request"""
        self.failure_count += 1
        self.last_failure_time = datetime.now()
        
        if self.state == CircuitState.HALF_OPEN:
            logger.warning("Circuit breaker reopening after failed recovery attempt")
            self.state = CircuitState.OPEN
            self.success_count = 0
        elif self.state == CircuitState.CLOSED:
            if self.failure_count >= self.failure_threshold:
                logger.error(f"Circuit breaker opening after {self.failure_count} failures")
                self.state = CircuitState.OPEN


class ResilientHttpClient:
    """HTTP client with retry logic and circuit breaker"""
    
    def __init__(
        self,
        base_url: str,
        max_retries: int = 3,
        timeout: float = 5.0,
        circuit_breaker: Optional[CircuitBreaker] = None
    ):
        """
        Initialize resilient HTTP client
        
        Args:
            base_url: Base URL for the service
            max_retries: Maximum number of retry attempts
            timeout: Request timeout in seconds
            circuit_breaker: Optional circuit breaker instance (creates default if None)
        """
        self.base_url = base_url.rstrip('/')
        self.max_retries = max_retries
        self.timeout = timeout
        self.circuit_breaker = circuit_breaker or CircuitBreaker()
    
    async def _calculate_backoff(self, attempt: int) -> float:
        """Calculate exponential backoff delay"""
        # Exponential backoff: 1s, 2s, 4s
        return min(2 ** attempt, 8)
    
    def _is_transient_error(self, error: Exception) -> bool:
        """Check if error is transient and should be retried"""
        if isinstance(error, (httpx.TimeoutException, httpx.ConnectError)):
            return True
        if isinstance(error, httpx.HTTPStatusError):
            # Retry on 5xx server errors and 429 rate limit
            return error.response.status_code >= 500 or error.response.status_code == 429
        return False
    
    async def request(
        self,
        method: str,
        path: str,
        **kwargs
    ) -> httpx.Response:
        """
        Make HTTP request with retry logic and circuit breaker
        
        Args:
            method: HTTP method (GET, POST, PUT, DELETE, etc.)
            path: Request path (will be appended to base_url)
            **kwargs: Additional arguments passed to httpx.request
        
        Returns:
            httpx.Response object
        
        Raises:
            httpx.HTTPError: If request fails after all retries
            CircuitBreakerOpenError: If circuit breaker is open
        """
        # Check circuit breaker
        if not self.circuit_breaker.can_attempt():
            raise CircuitBreakerOpenError(
                f"Circuit breaker is open for {self.base_url}"
            )
        
        url = f"{self.base_url}/{path.lstrip('/')}"
        last_error = None
        
        for attempt in range(self.max_retries + 1):
            try:
                async with httpx.AsyncClient(timeout=self.timeout) as client:
                    response = await client.request(method, url, **kwargs)
                    response.raise_for_status()
                    
                    # Success - record in circuit breaker
                    self.circuit_breaker.record_success()
                    return response
            
            except Exception as e:
                last_error = e
                
                # Check if we should retry
                if not self._is_transient_error(e):
                    # Non-transient error - fail immediately
                    self.circuit_breaker.record_failure()
                    logger.error(f"Non-transient error for {method} {url}: {e}")
                    raise
                
                # Check if we have retries left
                if attempt < self.max_retries:
                    backoff = await self._calculate_backoff(attempt)
                    logger.warning(
                        f"Transient error for {method} {url} (attempt {attempt + 1}/{self.max_retries + 1}): {e}. "
                        f"Retrying in {backoff}s..."
                    )
                    await asyncio.sleep(backoff)
                else:
                    # Out of retries
                    self.circuit_breaker.record_failure()
                    logger.error(
                        f"Request failed after {self.max_retries + 1} attempts for {method} {url}: {e}"
                    )
                    raise
        
        # Should not reach here, but just in case
        self.circuit_breaker.record_failure()
        raise last_error
    
    async def get(self, path: str, **kwargs) -> httpx.Response:
        """Make GET request"""
        return await self.request("GET", path, **kwargs)
    
    async def post(self, path: str, **kwargs) -> httpx.Response:
        """Make POST request"""
        return await self.request("POST", path, **kwargs)
    
    async def put(self, path: str, **kwargs) -> httpx.Response:
        """Make PUT request"""
        return await self.request("PUT", path, **kwargs)
    
    async def delete(self, path: str, **kwargs) -> httpx.Response:
        """Make DELETE request"""
        return await self.request("DELETE", path, **kwargs)
    
    async def patch(self, path: str, **kwargs) -> httpx.Response:
        """Make PATCH request"""
        return await self.request("PATCH", path, **kwargs)


class CircuitBreakerOpenError(Exception):
    """Raised when circuit breaker is open and request cannot be attempted"""
    pass
