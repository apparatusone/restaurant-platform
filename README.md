<div align="center">
  <h1>Restaurant POS + Kitchen Automation</h1>
  
  <p>
    A proof-of-concept microservice (with shared database) ecosystem with web-based POS frontend and optional ROS2-connected prep bot robot
  </p>
</div>

> [!WARNING]
> This project is a proof-of-concept. In its current state this project should **NOT** be used in a production environment.


### Key Features

* **API Gateway** - Single HTTP entrypoint with JWT validation and request proxying
* **Microservices Architecture** - Domain services for staff/auth, orders/checks, restaurant ops, payments, and automation
* **Shared Data Layer** - Common SQLAlchemy/MySQL foundation across Python services
* **Resilient Communication** - Inter-service HTTP clients with retry logic and circuit breaker pattern
* **Event-Driven Kitchen** - Redis pub/sub for kitchen order events
* **Robot Integration** - HTTP bridge to ROS2 for AprilTag-labeled ingredient automation
* **Multi-Tenancy** - Basic support for multiple restaurants

### How It Works

When a check is sent to the kitchen, the order domain publishes a Redis `kitchen.orders` event. The automation service subscribes, resolves which ingredients are robot-eligible (AprilTag-labeled), and drives the robot via an HTTP bridge into ROS2.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Built With

### Backend
* [FastAPI](https://fastapi.tiangolo.com/) - Modern Python web framework
* [SQLAlchemy](https://www.sqlalchemy.org/) - SQL toolkit and ORM
* [MySQL](https://www.mysql.com/) - Relational database
* [Redis](https://redis.io/) - In-memory data store for pub/sub
* [Pydantic](https://docs.pydantic.dev/) - Data validation

### Frontend
* [SvelteKit](https://kit.svelte.dev/) - Web application framework
* [TypeScript](https://www.typescriptlang.org/) - Typed JavaScript
* [Tailwind CSS](https://tailwindcss.com/) - Utility-first CSS framework

### Robotics
* [ROS2](https://docs.ros.org/en/jazzy/) - Robot Operating System
* [MoveIt](https://moveit.ros.org/) - Motion planning framework
* [AprilTag](https://april.eecs.umich.edu/software/apriltag) - Visual fiducial system
* Nucleo64 G474RE / Arduino

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Getting Started

### Prerequisites

* Python 3.10+
* Node.js 18+
* MySQL 8.0+
* Redis
* (Optional) ROS2 Jazzy for robot integration

### Basic Installation

For robot setup and ROS2 configuration, see the [robot documentation](my_robot/README.md).

---

1. Clone the repository
   ```sh
   git clone https://github.com/yourusername/restaurant-pos-automation.git
   cd restaurant-pos-automation
   ```

2. Set up environment variables
   ```sh
   cp .env.example .env
   # Edit .env with your configuration
   ```

3. Install Python dependencies
   ```sh
   pip install -r requirements.txt
   ```

4. Install frontend dependencies
   ```sh
   cd apps/pos-system
   npm install
   ```

5. Start the services
   ```sh
   # From project root
   ./start_services.sh
   ```

6. Access the POS system
   ```
   http://localhost:5173
   ```

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Usage

### Starting Services

```sh
# Start all backend services
./start_services.sh

# Stop all services
./stop_services.sh
```

### Service Endpoints

* **API Gateway**: `http://localhost:8000`
* **Staff Service**: `http://localhost:8001`
* **Order Service**: `http://localhost:8002`
* **Restaurant Service**: `http://localhost:8003`
* **Payment Service**: `http://localhost:8004`
* **Automation Service**: `http://localhost:8005`

### Running Tests

```sh
# Run tests for a specific service
cd services/staff-service
pytest

# Run all tests
pytest services/*/tests/
```

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Architecture

The system follows a microservices architecture with the following components:

### Services

* **API Gateway** (`port 8000`) - JWT validation, request routing, circuit breaker
* **Staff Service** (`port 8001`) - Authentication, staff management, timeclock
* **Order Service** (`port 8002`) - Menu items, checks, order items, recipes
* **Restaurant Service** (`port 8003`) - Tables, seatings, ingredients, kitchen operations
* **Payment Service** (`port 8004`) - Payment processing, receipts
* **Automation Service** (`port 8005`) - Robot task management, kitchen event subscription

### Communication Patterns

* **Synchronous**: HTTP/REST via resilient clients (retry + circuit breaker)
* **Asynchronous**: Redis pub/sub for kitchen events
* **Robot Bridge**: HTTP to ROS2 topics/services

For detailed architecture documentation, see [architecture.md](architecture.md).

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Roadmap

### Planned Additions

### Backend

- [ ] RBAC with Casbin
- [ ] Comprehensive test coverage
- [ ] Transaction rollback handling
- [ ] Database connection pooling configuration
- [ ] Metrics
- [ ] Idempotency keys for payments
- [ ] Database migrations (Alembic)
- [ ] print-service (for printing checks)

### Frontend
- [ ] Manager Dashboard
- [ ] Basic Robot control Dashboard
- [ ] This should be a local app (Tauri) that can store transactions if internet/etc is down

### Robot
- [ ] Joint/Task space control
- [ ] Real object detection and grasping
- [ ] Joint feedback (endstops or encoders)
- [ ] Automated homing routine
- [ ] switch from april tags for objects to opencv / 
 
<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Known Issues

* If the backend is down, the POS system is non-functional
* Using `Base.metadata.create_all()` which doesn't handle schema changes
* Tokens remain valid after account deletion
* Doesn't verify check totals against order service
* Services can't validate data from other services
* No retry logic if MySQL goes down temporarily
* Tax rate and other values are hardcoded
* CORS allows all origins
* No rate limiting
* No input sanitization
* Cross-service data access - doesn't fully align with microservice arch

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Acknowledgments

* [Best-README-Template](https://github.com/othneildrew/Best-README-Template)
* [FastAPI Documentation](https://fastapi.tiangolo.com/)
* [ROS2 Documentation](https://docs.ros.org/)
* [SvelteKit Documentation](https://kit.svelte.dev/)

<p align="right">(<a href="#readme-top">back to top</a>)</p>
