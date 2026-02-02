#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger, SetBool
from sensor_msgs.msg import JointState
import serial
import threading
import time

class HardwareInterface(Node):
    def __init__(self):
        super().__init__('hardware_interface')
        
        self.declare_parameter('port', '/dev/ttySTM32')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('reconnect_timeout', 1.0)
        self.declare_parameter('status_rate', 10.0)  # Hz for STATUS polling
        self.declare_parameter('default_gripper_open', [0.0225, -0.0225])
        self.declare_parameter('default_gripper_closed', [0.0, 0.0])
        self.declare_parameter('gripper_open_angle', 155)
        self.declare_parameter('gripper_close_angle', 85)
        
        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.reconnect_timeout = self.get_parameter('reconnect_timeout').value
        self.status_rate = self.get_parameter('status_rate').value
        self.default_gripper_open = self.get_parameter('default_gripper_open').value
        self.default_gripper_closed = self.get_parameter('default_gripper_closed').value
        self.gripper_open_angle = self.get_parameter('gripper_open_angle').value
        self.gripper_close_angle = self.get_parameter('gripper_close_angle').value
        
        self.command_subscription = self.create_subscription(
            String,
            'motor_command',
            self.command_callback,
            10
        )
        
        self.feedback_publisher = self.create_publisher(String, 'motor_feedback', 10)
        
        # Use BEST_EFFORT QoS to match robot_state_publisher and move_group subscribers
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        joint_state_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', joint_state_qos)
        
        self.serial_port = None
        self.serial_lock = threading.Lock()
        self._status_paused = False  # External control of STATUS polling
        self._command_lock = threading.Lock()  # Prevent STATUS during commands
        
        # Track last known joint positions
        self._last_joint_positions = [0.0, 0.0, 0.0] + self.default_gripper_open
        
        # Homed state tracking
        self._is_homed = False
        
        # Service to query homed status
        self.create_service(Trigger, '/robot_homed_status', self.handle_homed_status_request)
        
        # Service to pause/resume STATUS polling
        self.create_service(SetBool, '/hardware_interface/pause_status', self.handle_pause_status)
        
        # Service to control gripper
        from std_srvs.srv import SetBool as GripperSrv
        self.create_service(GripperSrv, '/hardware_interface/gripper', self.handle_gripper_command)
        
        self.connect_serial()
        
        # Publish joint states at 10Hz like old simple_pick_node did
        self.create_timer(0.1, self.publish_joint_states_timer)
        
        self.running = True
        self.read_thread = threading.Thread(target=self.read_serial, daemon=True)
        self.read_thread.start()
        
        # Timer to periodically request STATUS
        if self.status_rate > 0:
            self.create_timer(1.0 / self.status_rate, self.request_status)
        
        # Query homed state on startup (after short delay for serial to settle)
        self._homed_query_timer = self.create_timer(1.0, self.query_homed_state_once)
        
        self.get_logger().info('Hardware Interface started')

    def connect_serial(self):
        with self.serial_lock:
            if self.serial_port and self.serial_port.is_open:
                try:
                    self.serial_port.close()
                except:
                    pass
            
            try:
                self.serial_port = serial.Serial(self.port, self.baudrate, timeout=1)
                self.get_logger().info(f'Connected to {self.port} at {self.baudrate} baud')
                
                # Query homed state after reconnect (500ms delay for STM32 to be ready)
                self._reconnect_query_timer = self.create_timer(0.5, self._query_after_reconnect)
                
                return True
            except serial.SerialException as e:
                self.get_logger().error(f'Failed to connect to {self.port}: {e}')
                self.serial_port = None
                return False
    
    def _query_after_reconnect(self):
        """Query homed state after reconnect delay"""
        self.query_homed_state()
        # Cancel timer after first run
        if hasattr(self, '_reconnect_query_timer'):
            self._reconnect_query_timer.cancel()

    def command_callback(self, msg):
        command = msg.data
        
        with self._command_lock:
            # Query homed state after HOME command completes
            if command == 'HOME':
                self._query_homed_after_done = True
            
            with self.serial_lock:
                if self.serial_port and self.serial_port.is_open:
                    try:
                        self.serial_port.write(f"{command}\n".encode('utf-8'))
                        self.serial_port.flush()
                    except serial.SerialException as e:
                        self.get_logger().error(f'Serial write error: {e}')
                        self.connect_serial()
    
    def request_status(self):
        """Periodically request STATUS from STM32."""
        # Don't poll if externally paused
        if self._status_paused:
            return
        
        # Don't send if a command is being processed
        if not self._command_lock.acquire(blocking=False):
            return
        
        try:
            with self.serial_lock:
                if self.serial_port and self.serial_port.is_open:
                    try:
                        self.serial_port.write(b"STATUS\n")
                        self.serial_port.flush()
                    except serial.SerialException as e:
                        self.get_logger().error(f'Serial write error: {e}')
                        self.connect_serial()
        finally:
            self._command_lock.release()

    def read_serial(self):
        buffer = ""
        while self.running and rclpy.ok():
            with self.serial_lock:
                port_ok = self.serial_port and self.serial_port.is_open
            
            if port_ok:
                try:
                    # Read without holding lock for long
                    with self.serial_lock:
                        if self.serial_port and self.serial_port.in_waiting > 0:
                            data = self.serial_port.read(self.serial_port.in_waiting).decode('utf-8', errors='ignore')
                            buffer += data
                    
                    # Process complete lines outside the lock
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        if line:
                            # Publish feedback to topic
                            msg = String()
                            msg.data = line
                            self.feedback_publisher.publish(msg)
                            
                            # Handle DONE - request fresh joint state and check for homed query
                            if line == 'DONE' or line == 'STOPPED':
                                self.serial_port.write(b"STATUS\n")
                                self.serial_port.flush()
                                if getattr(self, '_query_homed_after_done', False):
                                    self._query_homed_after_done = False
                                    self.query_homed_state()
                            
                            # Handle HOMED/NOT_HOMED responses
                            if line == 'HOMED':
                                self._is_homed = True
                            elif line == 'NOT_HOMED':
                                self._is_homed = False
                            
                            # Parse POS messages and publish JointState
                            if line.startswith('POS '):
                                self.parse_and_publish_joint_state(line)
                    
                    time.sleep(0.01)
                except serial.SerialException as e:
                    self.get_logger().error(f'Serial read error: {e}')
                    buffer = ""
                    time.sleep(1)
                    self.connect_serial()
                except Exception:
                    pass
            else:
                time.sleep(self.reconnect_timeout)
                self.connect_serial()

    def destroy_node(self):
        self.running = False
        with self.serial_lock:
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()
        super().destroy_node()
    
    def parse_and_publish_joint_state(self, line):
        """Parse 'POS j0 j1 j2 j3' and update internal state.
        STM32 sends: j0=linear, j1=rotation(always 0), j2=shoulder, j3=elbow
        We track: linear, shoulder, elbow, gripper_left, gripper_right
        """
        try:
            parts = line.split()
            if len(parts) < 5:  # POS + 4 joint values
                self.get_logger().warn(f'Incomplete POS message: {line} (expected 5 parts, got {len(parts)})')
                return
                
            stm32_values = [float(parts[i]) for i in range(1, 5)]
            
            # Update last known positions (map: linear=j0, shoulder=j2, elbow=j3)
            self._last_joint_positions[0] = stm32_values[0]
            self._last_joint_positions[1] = stm32_values[2]
            self._last_joint_positions[2] = stm32_values[3]
            # Gripper positions remain at default (0.02, -0.02) unless we get gripper feedback
            
        except (ValueError, IndexError) as e:
            self.get_logger().warn(f'Failed to parse POS message: {line} - {e}')
    
    def publish_joint_states_timer(self):
        """Publish joint states at 10Hz using last known positions."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['linear_joint', 'shoulder_joint', 'elbow_joint', 'gripper_left_joint', 'gripper_right_joint']
        msg.position = self._last_joint_positions.copy()
        
        self.joint_state_publisher.publish(msg)
    
    def query_homed_state_once(self):
        """One-shot timer to query homed state on startup."""
        self.query_homed_state()
        # Cancel timer after first run
        self._homed_query_timer.cancel()
    
    def query_homed_state(self):
        """Send HOMED query to STM32."""
        with self.serial_lock:
            if self.serial_port and self.serial_port.is_open:
                try:
                    self.serial_port.write(b"HOMED\n")
                    self.serial_port.flush()
                except serial.SerialException as e:
                    self.get_logger().error(f'Serial write error: {e}')
    
    def handle_homed_status_request(self, request, response):
        """Handle homed status service request."""
        response.success = self._is_homed
        response.message = 'Robot is homed' if self._is_homed else 'Robot not homed'
        return response
    
    def handle_pause_status(self, request, response):
        """Handle pause/resume STATUS polling service request."""
        self._status_paused = request.data
        response.success = True
        response.message = 'STATUS polling paused' if request.data else 'STATUS polling resumed'
        return response
    
    def handle_gripper_command(self, request, response):
        """Handle gripper control service request.
        
        Args:
            request.data: True = close gripper, False = open gripper
        """
        angle = self.gripper_close_angle if request.data else self.gripper_open_angle
        
        with self.serial_lock:
            if self.serial_port and self.serial_port.is_open:
                try:
                    cmd = f'GRIPPER {angle}\n'
                    self.serial_port.write(cmd.encode())
                    self.serial_port.flush()
                    response.success = True
                    response.message = f'Gripper {"closed" if request.data else "opened"} ({angle}°)'
                except serial.SerialException as e:
                    self.get_logger().error(f'Serial write error: {e}')
                    response.success = False
                    response.message = str(e)
            else:
                response.success = False
                response.message = 'Serial port not open'
        
        return response

def main(args=None):
    rclpy.init(args=args)
    node = HardwareInterface()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()
