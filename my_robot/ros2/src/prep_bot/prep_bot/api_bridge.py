#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
from flask import Flask, request, jsonify
from flask_cors import CORS
import threading
import json
import requests
import time

class ApiBridge(Node):
    def __init__(self):
        super().__init__('api_bridge')
        # Publisher to send orders to kitchen controller
        self.order_publisher = self.create_publisher(String, 'new_order', 10)
        
        # Publisher for motor commands (to hardware_interface)
        self.motor_cmd_publisher = self.create_publisher(String, 'motor_command', 10)
        
        # Publisher for pick commands (to task_controller)
        self.pick_cmd_publisher = self.create_publisher(String, '/pick_command', 10)
        
        # Subscribe to robot status updates
        self.status_subscription = self.create_subscription(
            String,
            'robot_status',
            self.status_callback,
            10
        )
        
        # Subscribe to motor feedback (from hardware_interface)
        self.motor_feedback_subscription = self.create_subscription(
            String,
            'motor_feedback',
            self.motor_feedback_callback,
            10
        )
        
        # Subscribe to pick status (from task_controller)
        self.pick_status_subscription = self.create_subscription(
            String,
            '/pick_status',
            self.pick_status_callback,
            10
        )
        
        # Service clients for status queries
        self.calibration_client = self.create_client(Trigger, '/camera_calibration_status')
        self.homed_client = self.create_client(Trigger, '/robot_homed_status')
        
        # Cached status (updated periodically)
        self._cached_calibrated = False
        
        # Poll status every 2 seconds
        # self.create_timer(2.0, self.update_cached_status)
        
        # Track current robot status
        self.current_status = 'IDLE'
        self.status_message = 'Robot is ready'
        self.previous_status = 'IDLE'
        self.last_motor_feedback = None
        self.last_pick_status = None
        self.pick_result = None
        
        # Backend configuration
        self.declare_parameter('backend_url', 'http://10.0.1.194:8000')
        self.backend_url = self.get_parameter('backend_url').value
        
        self.get_logger().info('API Bridge started')

    def status_callback(self, msg):
        """Receive status updates from kitchen controller"""
        self.previous_status = self.current_status
        self.current_status = msg.data
        self.get_logger().info(f'Robot status updated: {self.current_status}')
        
        # Update status message based on state
        status_messages = {
            'IDLE': 'Robot is ready',
            'MOVING_TO_BIN_1': 'Moving to source bin',
            'PICKING': 'Picking ingredient',
            'MOVING_TO_BIN_2': 'Moving to destination bin',
            'PLACING': 'Placing ingredient',
            'RESETTING': 'Returning to home position',
            'ERROR': 'Robot encountered an error'
        }
        self.status_message = status_messages.get(self.current_status, 'Unknown state')
        
        # Check if robot just became IDLE (completed an order)
        if self.current_status == 'IDLE' and self.previous_status != 'IDLE':
            self.get_logger().info('Order completed, checking for pending orders in 3 seconds...')
            # Start a thread to process next order after delay
            threading.Thread(target=self.process_next_order_delayed, daemon=True).start()

    def motor_feedback_callback(self, msg):
        """Receive feedback from hardware_interface"""
        self.last_motor_feedback = msg.data
        self.get_logger().debug(f'Motor feedback received: {msg.data}')

    def pick_status_callback(self, msg):
        """Receive status from task_controller"""
        self.last_pick_status = msg.data
        self.get_logger().info(f'Pick status received: {msg.data}')
        
        # Handle IDLE status (recovery complete or home complete)
        if msg.data == 'IDLE':
            self.pick_result = {'status': 'idle'}
            return
        
        # Parse status: "SUCCESS <cube_id>" or "FAILED <cube_id> <reason>"
        parts = msg.data.split()
        if len(parts) >= 2:
            status = parts[0]
            cube_id = parts[1]
            reason = parts[2] if len(parts) > 2 else None
            
            # Handle "SUCCESS None" from HOME command
            if cube_id == 'None':
                self.pick_result = {'status': 'idle'}
                return
            
            self.pick_result = {
                'status': status.lower(),
                'cube_id': int(cube_id),
                'reason': reason
            }
    
    def update_cached_status(self):
        """Periodically update cached status from services (runs in ROS thread)"""
        # Query calibration status
        if self.calibration_client.service_is_ready():
            future = self.calibration_client.call_async(Trigger.Request())
            future.add_done_callback(self._calibration_callback)
    
    def _calibration_callback(self, future):
        """Handle calibration status response"""
        try:
            response = future.result()
            self._cached_calibrated = response.success
        except Exception as e:
            self.get_logger().debug(f'Failed to get calibration status: {e}')

    def process_order(self, order_data):
        """Receive order from FastAPI and publish to ROS2 topic"""
        msg = String()
        msg.data = json.dumps(order_data)
        self.order_publisher.publish(msg)
        self.get_logger().info(f'Published order: {order_data}')
    
    def get_status(self):
        """Return current robot status for API"""
        is_busy = self.current_status != 'IDLE'
        return {
            'status': self.current_status.lower(),
            'message': self.status_message,
            'is_busy': is_busy
        }
    
    def process_next_order_delayed(self):
        """Wait 3 seconds then trigger processing of next pending order"""
        try:
            time.sleep(3)
            self.get_logger().info('Requesting backend to process next pending order...')
            
            response = requests.post(
                f'{self.backend_url}/robot/queue/process-pending',
                timeout=5
            )
            
            if response.status_code == 200:
                result = response.json()
                self.get_logger().info(f'Backend response: {result}')
            elif response.status_code == 404:
                self.get_logger().info('No pending orders in queue')
            else:
                self.get_logger().warn(f'Unexpected response: {response.status_code} - {response.text}')
                
        except requests.exceptions.ConnectionError:
            self.get_logger().warn(f'Could not connect to backend at {self.backend_url}')
        except requests.exceptions.Timeout:
            self.get_logger().warn('Backend request timed out')
        except Exception as e:
            self.get_logger().error(f'Error processing next order: {str(e)}')

# Flask app to receive HTTP requests from FastAPI
app = Flask(__name__)
CORS(app)

def get_node():
    """Get ROS node from Flask config with error checking."""
    node = app.config.get('ros_node')
    if node is None:
        raise RuntimeError("ROS node not initialized")
    return node

@app.route('/health', methods=['GET'])
def health():
    return jsonify({"status": "healthy"}), 200

@app.route('/status', methods=['GET'])
def get_status():
    try:
        status_data = get_node().get_status()
        return jsonify(status_data), 200
    except Exception as e:
        return jsonify({
            "status": "unknown",
            "message": f"Unable to get robot status: {str(e)}",
            "is_busy": False
        }), 500


@app.route('/robot/status', methods=['GET'])
def get_robot_ready_status():
    """Get robot ready state (homed + calibrated)"""
    try:
        node = get_node()
        
        # Query homed status (returns cached state from hardware_interface)
        homed = False
        if node.homed_client.service_is_ready():
            future = node.homed_client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=1.0)
            if future.done():
                homed = future.result().success
        
        # Query calibration status (returns cached state from camera_calibration)
        calibrated = False
        if node.calibration_client.service_is_ready():
            future = node.calibration_client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=1.0)
            if future.done():
                calibrated = future.result().success
        
        return jsonify({
            "homed": homed,
            "calibrated": calibrated,
            "ready": homed and calibrated
        }), 200
    except Exception as e:
        return jsonify({
            "homed": False,
            "calibrated": False,
            "ready": False,
            "error": str(e)
        }), 500

@app.route('/robot/command', methods=['POST'])
def robot_command():
    """Send raw command to microcontroller via hardware_interface"""
    try:
        node = get_node()
        data = request.json
        cmd = data.get('command', '')
        if not cmd:
            return jsonify({"status": "error", "message": "No command provided"}), 400
        
        # Clear any stale feedback
        node.last_motor_feedback = None
        
        # Publish to motor_command topic
        msg = String()
        msg.data = cmd
        node.motor_cmd_publisher.publish(msg)
        node.get_logger().info(f'Sent command: {cmd}')
        
        # Different timeouts for different commands
        if cmd.startswith('JOINTS') or cmd == 'GO_HOME':
            timeout = 10.0  # Movement commands take longer
        elif cmd == 'STATUS':
            timeout = 1.0  # STATUS should be quick
        elif cmd.startswith('WAYPOINT') or cmd == 'START_TRAJ' or cmd == 'END_TRAJ' or cmd == 'CLEAR_TRAJ':
            timeout = 0.5  # Trajectory commands are quick
        else:
            timeout = 2.0  # Other commands
        
        # Poll for response with shorter intervals
        # Main thread's spin() handles callbacks and sets last_motor_feedback
        start_time = time.time()
        poll_interval = 0.05  # Check every 50ms
        
        while (time.time() - start_time) < timeout:
            if node.last_motor_feedback:
                response = node.last_motor_feedback
                node.last_motor_feedback = None  # Clear after reading
                return jsonify({"status": "success", "response": response}), 200
            time.sleep(poll_interval)
        
        # Timeout - no response received
        return jsonify({"status": "error", "message": f"No response from Arduino after {timeout}s"}), 503
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500

@app.route('/robot/info', methods=['GET'])
def robot_info():
    """Get robot configuration from URDF"""
    try:
        from urdf_parser_py.urdf import URDF
        from ament_index_python.packages import get_package_share_directory
        import os
        
        # Find URDF file in prep_bot package
        pkg_share = get_package_share_directory('prep_bot')
        urdf_path = os.path.join(pkg_share, 'urdf', 'robot.urdf')
        robot = URDF.from_xml_file(urdf_path)
        
        # Extract joint info
        joints = []
        for joint in robot.joints:
            if joint.type in ['revolute', 'prismatic', 'continuous']:
                info = {
                    'name': joint.name,
                    'type': joint.type,
                    'min': joint.limit.lower if joint.limit else None,
                    'max': joint.limit.upper if joint.limit else None,
                }
                joints.append(info)
        
        return jsonify({
            'name': robot.name,
            'urdf_url': '/robot/robot.urdf',
            'joints': joints,
            'gripper': {'min': 0, 'max': 155, 'unit': 'degrees'}
        }), 200
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500


@app.route('/robot/objects', methods=['GET'])
def get_objects():
    """Get detected objects from scene_manager"""
    try:
        node = get_node()
        import json
        from std_srvs.srv import Trigger
        
        # Create a separate client for this request
        client = node.create_client(Trigger, '/scene_manager/get_objects')
        if not client.wait_for_service(timeout_sec=1.0):
            return jsonify({"status": "error", "message": "scene_manager service not available"}), 503
        
        request = Trigger.Request()
        future = client.call_async(request)
        
        # Use a simple timeout loop
        import time
        start_time = time.time()
        while not future.done() and (time.time() - start_time) < 2.0:
            time.sleep(0.05)
        
        if not future.done():
            return jsonify({"status": "error", "message": "Service call timeout"}), 503
        
        response = future.result()
        if response.success:
            objects = json.loads(response.message)
            return jsonify({"status": "success", "objects": objects}), 200
        else:
            return jsonify({"status": "error", "message": "Failed to get objects"}), 500
            
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500


@app.route('/robot/pick_cube', methods=['POST'])
def pick_cube():
    """Pick a cube by AprilTag ID and place in assembly bin"""
    try:
        node = get_node()
        data = request.json
        cube_tag = data.get('cube_apriltag_id')
        
        if cube_tag is None:
            return jsonify({
                "status": "error",
                "reason": "missing_parameter: cube_apriltag_id required"
            }), 400
        
        node.get_logger().info(f'Pick cube request: tag={cube_tag}')
        
        # Clear previous result
        node.pick_result = None
        
        # Send pick command to task_controller
        msg = String()
        msg.data = f'PICK {cube_tag}'
        node.pick_cmd_publisher.publish(msg)
        
        # Wait for task_controller to report success/failure
        # Main thread's spin() handles callbacks and sets pick_result
        timeout = 120.0  # 2 minutes for full pick sequence
        start_time = time.time()
        poll_interval = 0.1
        
        while (time.time() - start_time) < timeout:
            if node.pick_result:
                result = node.pick_result
                node.pick_result = None  # Clear for next request
                
                if result['status'] == 'success':
                    return jsonify({
                        "status": "success",
                        "cube_id": result['cube_id']
                    }), 200
                elif result['status'] == 'failed':
                    # Wait for recovery to complete (IDLE status)
                    failure_reason = result.get('reason', 'unknown')
                    node.get_logger().info(f'Pick failed, waiting for recovery...')
                    
                    recovery_timeout = 30.0  # 30 seconds for recovery
                    recovery_start = time.time()
                    
                    while (time.time() - recovery_start) < recovery_timeout:
                        if node.pick_result and node.pick_result.get('status') == 'idle':
                            node.pick_result = None
                            node.get_logger().info('Recovery complete, returning error')
                            return jsonify({
                                "status": "error",
                                "reason": failure_reason
                            }), 200
                        
                        time.sleep(poll_interval)
                    
                    # Recovery timeout - return anyway
                    node.get_logger().warn('Recovery timeout, returning error')
                    return jsonify({
                        "status": "error",
                        "reason": failure_reason
                    }), 200
            
            time.sleep(poll_interval)
        
        # Timeout
        return jsonify({
            "status": "error",
            "reason": "timeout"
        }), 503
        
    except Exception as e:
        get_node().get_logger().error(f'Pick cube error: {str(e)}')
        return jsonify({
            "status": "error",
            "reason": f"exception: {str(e)}"
        }), 500


@app.route('/robot/home', methods=['POST'])
def robot_home():
    """Send robot to full ready position (including linear retract).
    
    Called by automation-service after order completes.
    """
    try:
        node = get_node()
        node.get_logger().info('Home request received')
        
        # Clear previous result
        node.pick_result = None
        
        # Send HOME command to task_controller
        msg = String()
        msg.data = 'HOME'
        node.pick_cmd_publisher.publish(msg)
        
        # Wait for completion
        timeout = 30.0
        start_time = time.time()
        poll_interval = 0.1
        
        while (time.time() - start_time) < timeout:
            if node.pick_result and node.pick_result.get('status') == 'idle':
                node.pick_result = None
                return jsonify({"status": "success"}), 200
            time.sleep(poll_interval)
        
        return jsonify({"status": "error", "reason": "timeout"}), 503
        
    except Exception as e:
        get_node().get_logger().error(f'Home error: {str(e)}')
        return jsonify({"status": "error", "reason": str(e)}), 500


def run_flask():
    app.run(host='0.0.0.0', port=5001, debug=False)

def main():
    rclpy.init()
    node = ApiBridge()
    
    # Store node in Flask config
    app.config['ros_node'] = node
    
    # Run Flask in separate thread
    flask_thread = threading.Thread(target=run_flask, daemon=True)
    flask_thread.start()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()