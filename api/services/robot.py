import httpx
import logging
from typing import Dict, Any, List, Optional, TYPE_CHECKING
from datetime import datetime
from sqlalchemy.orm import Session
import json
import os

if TYPE_CHECKING:
    from ..models.robot_queue import RobotQueue

logger = logging.getLogger(__name__)

class RobotKitchenService:
    def __init__(self):
        # Configuration for ROS2 PC connection
        self.ros2_url = os.getenv('ROS2_PC_URL', 'http://10.0.1.66:5001')
        self.timeout = 10.0
        self.max_retries = 3
    
    def add_to_queue(self, db: Session, order_id: int) -> 'RobotQueue':
        """Add an order to the robot queue"""
        from ..models.robot_queue import RobotQueue, RobotQueueStatus
        
        # Check if already in queue
        existing = db.query(RobotQueue).filter(RobotQueue.order_id == order_id).first()
        if existing:
            logger.info(f"Order {order_id} already in robot queue with status {existing.status.value}")
            return existing
        
        # Create new queue entry
        queue_entry = RobotQueue(
            order_id=order_id,
            status=RobotQueueStatus.PENDING
        )
        db.add(queue_entry)
        db.commit()
        db.refresh(queue_entry)
        logger.info(f"Added order {order_id} to robot queue")
        return queue_entry
    
    def update_queue_status(
        self, 
        db: Session, 
        order_id: int, 
        status: str,
        error_message: Optional[str] = None,
        robot_response: Optional[Dict] = None
    ) -> Optional['RobotQueue']:
        """Update the status of an order in the queue"""
        from ..models.robot_queue import RobotQueue, RobotQueueStatus
        
        queue_entry = db.query(RobotQueue).filter(RobotQueue.order_id == order_id).first()
        if not queue_entry:
            logger.warning(f"Queue entry not found for order {order_id}")
            return None
        
        queue_entry.status = RobotQueueStatus[status.upper()]
        queue_entry.last_attempt_at = datetime.utcnow()
        
        if status.upper() == "SENT":
            queue_entry.sent_at = datetime.utcnow()
        elif status.upper() == "CONFIRMED":
            queue_entry.confirmed_at = datetime.utcnow()
        elif status.upper() == "FAILED":
            queue_entry.retry_count += 1
            queue_entry.error_message = error_message
        
        if robot_response:
            queue_entry.robot_response = json.dumps(robot_response)
        
        db.commit()
        db.refresh(queue_entry)
        return queue_entry
    
    def get_queue_status(self, db: Session, order_id: int) -> Optional[Dict[str, Any]]:
        """Get the queue status for an order"""
        from ..models.robot_queue import RobotQueue
        
        queue_entry = db.query(RobotQueue).filter(RobotQueue.order_id == order_id).first()
        if not queue_entry:
            return None
        
        return {
            "order_id": queue_entry.order_id,
            "status": queue_entry.status.value,
            "retry_count": queue_entry.retry_count,
            "last_attempt_at": queue_entry.last_attempt_at.isoformat() if queue_entry.last_attempt_at else None,
            "sent_at": queue_entry.sent_at.isoformat() if queue_entry.sent_at else None,
            "confirmed_at": queue_entry.confirmed_at.isoformat() if queue_entry.confirmed_at else None,
            "error_message": queue_entry.error_message,
            "robot_response": json.loads(queue_entry.robot_response) if queue_entry.robot_response else None,
            "created_at": queue_entry.created_at.isoformat()
        }
    
    def get_pending_orders(self, db: Session, limit: int = 10) -> List['RobotQueue']:
        """Get pending orders from the queue"""
        from ..models.robot_queue import RobotQueue, RobotQueueStatus
        
        return db.query(RobotQueue)\
            .filter(RobotQueue.status == RobotQueueStatus.PENDING)\
            .limit(limit)\
            .all()
    
    async def send_order_to_robot(
        self, 
        order_id: int, 
        items: List[Dict[str, Any]],
        db: Optional[Session] = None
    ) -> Dict[str, Any]:
        """
        Send order information to the robot kitchen via ROS2 bridge
        
        Args:
            order_id: The order ID from the database
            items: List of order items with their details
            db: Optional database session for queue management
            
        Returns:
            Response from the robot kitchen system
        """
        try:
            payload = {
                "order_id": order_id,
                "items": items
            }
            
            # Update queue status to indicate attempt
            if db:
                self.update_queue_status(db, order_id, "PENDING")
            
            async with httpx.AsyncClient(timeout=self.timeout) as client:
                response = await client.post(
                    f"{self.ros2_url}/order",
                    json=payload
                )
                response.raise_for_status()
                result = response.json()
                logger.info(f"Successfully sent order {order_id} to robot kitchen")
                
                # Update queue status to sent/confirmed
                if db:
                    if result.get("status") == "success":
                        self.update_queue_status(db, order_id, "CONFIRMED", robot_response=result)
                    else:
                        self.update_queue_status(db, order_id, "SENT", robot_response=result)
                
                return result
                
        except httpx.TimeoutException as e:
            error_msg = "Robot kitchen connection timeout"
            logger.error(f"Timeout sending order {order_id} to robot kitchen")
            if db:
                self.update_queue_status(db, order_id, "FAILED", error_message=error_msg)
            return {"status": "error", "message": error_msg}
        except httpx.HTTPError as e:
            error_msg = f"HTTP error: {str(e)}"
            logger.error(f"HTTP error sending order {order_id} to robot kitchen: {str(e)}")
            if db:
                self.update_queue_status(db, order_id, "FAILED", error_message=error_msg)
            return {"status": "error", "message": error_msg}
        except Exception as e:
            error_msg = str(e)
            logger.error(f"Error sending order {order_id} to robot kitchen: {str(e)}")
            if db:
                self.update_queue_status(db, order_id, "FAILED", error_message=error_msg)
            return {"status": "error", "message": error_msg}
    
    async def check_robot_health(self) -> bool:
        """Check if the robot kitchen system is available"""
        try:
            async with httpx.AsyncClient(timeout=5.0) as client:
                response = await client.get(f"{self.ros2_url}/health")
                return response.status_code == 200
        except Exception as e:
            logger.warning(f"Robot kitchen health check failed: {str(e)}")
            return False
    
    async def get_robot_status(self) -> Dict[str, Any]:
        """Get the current status of the robot kitchen"""
        try:
            async with httpx.AsyncClient(timeout=5.0) as client:
                response = await client.get(f"{self.ros2_url}/status")
                if response.status_code == 200:
                    return response.json()
                else:
                    return {"status": "unknown", "message": "Unable to get robot status"}
        except Exception as e:
            logger.warning(f"Failed to get robot status: {str(e)}")
            return {"status": "offline", "message": str(e)}
    
    async def is_robot_busy(self) -> bool:
        """Check if the robot is currently busy processing an order"""
        status = await self.get_robot_status()
        # Check both 'status' and 'state' fields, consider idle/ready/offline as not busy
        robot_state = status.get("status", status.get("state", "unknown")).lower()
        return robot_state not in ["idle", "ready", "offline", "unknown"]

# Singleton instance
robot_service = RobotKitchenService()
