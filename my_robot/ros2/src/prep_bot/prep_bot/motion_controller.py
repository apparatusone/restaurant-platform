#!/usr/bin/env python3
"""
Motion Controller

MoveIt interface for motion planning and execution.
Action server that wraps MoveIt's move_group action.
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import (
    CollisionObject, Constraints, PositionConstraint, 
    OrientationConstraint, BoundingVolume,
    JointConstraint
)
from moveit_msgs.action import MoveGroup
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, ReliabilityPolicy


class MotionController(Node):
    def __init__(self):
        super().__init__('motion_controller')
        
        # Declare parameters with defaults
        self.declare_parameter('planning_attempts', 10)
        self.declare_parameter('planning_time', 5.0)
        self.declare_parameter('velocity_scaling', 0.12)
        self.declare_parameter('acceleration_scaling', 0.12)
        self.declare_parameter('position_tolerance', 0.005)
        self.declare_parameter('execute_motion', False)  # Safety: disabled by default
        
        # Get parameter values
        self.planning_attempts = self.get_parameter('planning_attempts').value
        self.planning_time = self.get_parameter('planning_time').value
        self.velocity_scaling = self.get_parameter('velocity_scaling').value
        self.acceleration_scaling = self.get_parameter('acceleration_scaling').value
        self.position_tolerance = self.get_parameter('position_tolerance').value
        self.execute_motion = self.get_parameter('execute_motion').value
        
        # MoveIt action client
        self.move_action_client = ActionClient(self, MoveGroup, '/move_action')
        
        # Action server for external clients (reuse MoveGroup action)
        self.action_server = ActionServer(
            self,
            MoveGroup,
            'motion_controller/move_group',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        
        # Joint state tracking
        self.joint_names = ['linear_joint', 'shoulder_joint', 'elbow_joint', 
                           'gripper_left_joint', 'gripper_right_joint']
        self.current_joint_positions = [0.0, 0.0, 0.0, 0.02, -0.02]  # Gripper open
        
        # Subscribe to joint_states with BEST_EFFORT QoS
        joint_state_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            JointState, '/joint_states', 
            self.joint_state_callback, joint_state_qos
        )
        
        # State
        self.current_moveit_goal_handle = None
        
        exec_status = "ENABLED" if self.execute_motion else "DISABLED"
        self.get_logger().info(f'Motion Controller Ready (execution: {exec_status})')
        self.get_logger().info(f'Planning: {self.planning_attempts} attempts, '
                              f'{self.planning_time}s timeout')
        self.get_logger().info(f'Scaling: velocity={self.velocity_scaling}, '
                              f'accel={self.acceleration_scaling}')
    
    def joint_state_callback(self, msg):
        """Update internal joint state from robot feedback."""
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                if idx < len(msg.position):
                    self.current_joint_positions[i] = msg.position[idx]
    
    def goal_callback(self, goal_request):
        """Accept or reject incoming action goals."""
        self.get_logger().info('Received motion goal')
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle):
        """Handle cancellation requests."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT
    
    async def execute_callback(self, goal_handle):
        """Execute the MoveGroup action - just forward to MoveIt."""
        self.get_logger().info('Executing motion goal')
        
        # Forward goal directly to MoveIt
        moveit_goal = goal_handle.request
        
        # Override with our configured parameters
        moveit_goal.request.num_planning_attempts = self.planning_attempts
        moveit_goal.request.allowed_planning_time = self.planning_time
        moveit_goal.request.max_velocity_scaling_factor = self.velocity_scaling
        moveit_goal.request.max_acceleration_scaling_factor = self.acceleration_scaling
        moveit_goal.planning_options.plan_only = not self.execute_motion
        
        if not self.move_action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('MoveIt action server not available!')
            goal_handle.abort()
            return goal_handle.request
        
        # Send to MoveIt
        send_goal_future = self.move_action_client.send_goal_async(moveit_goal)
        await send_goal_future
        
        moveit_goal_handle = send_goal_future.result()
        if not moveit_goal_handle or not moveit_goal_handle.accepted:
            self.get_logger().error('MoveIt goal rejected!')
            goal_handle.abort()
            return goal_handle.request
        
        self.get_logger().info('MoveIt goal accepted, planning...')
        
        # Wait for result
        result_future = moveit_goal_handle.get_result_async()
        await result_future
        
        result = result_future.result()
        
        if result.result.error_code.val == 1:
            self.get_logger().info('✓ Motion succeeded!')
            goal_handle.succeed()
        else:
            self.get_logger().error(f'Motion failed: error code {result.result.error_code.val}')
            goal_handle.abort()
        
        return result.result


def main(args=None):
    rclpy.init(args=args)
    node = MotionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
