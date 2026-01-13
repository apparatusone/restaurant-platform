#!/usr/bin/env python3
"""
Motion Controller

MoveIt interface for motion planning and execution.
Provides high-level methods for moving to poses, attaching/detaching objects.
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import (
    CollisionObject, Constraints, PositionConstraint, 
    OrientationConstraint, BoundingVolume, AttachedCollisionObject,
    JointConstraint
)
from moveit_msgs.action import MoveGroup
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
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
        
        # Publishers
        self.attached_collision_pub = self.create_publisher(
            AttachedCollisionObject, '/attached_collision_object', 10
        )
        self.motor_cmd_pub = self.create_publisher(String, '/motor_command', 10)
        
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
        
        # Subscribe to motion commands
        self.create_subscription(
            PoseStamped, '/motion/target_pose',
            self.target_pose_callback, 10
        )
        
        # State
        self.current_goal_handle = None
        self.motion_in_progress = False
        
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
    
    def target_pose_callback(self, msg):
        """Handle incoming target pose requests."""
        if self.motion_in_progress:
            self.get_logger().warn('Motion already in progress, ignoring request')
            return
        
        self.plan_and_move(msg)
    
    def plan_and_move(self, target_pose, position_tolerance=None):
        """
        Plan and execute motion to target pose.
        
        Args:
            target_pose: PoseStamped target
            position_tolerance: Override default position tolerance (meters)
        """
        if not self.move_action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('MoveIt action server not available!')
            return False
        
        goal = MoveGroup.Goal()
        goal.request.group_name = 'robot_arm'
        goal.request.num_planning_attempts = self.planning_attempts
        goal.request.allowed_planning_time = self.planning_time
        goal.request.max_velocity_scaling_factor = self.velocity_scaling
        goal.request.max_acceleration_scaling_factor = self.acceleration_scaling
        
        # Position constraint
        constraints = Constraints()
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = 'base_link'
        pos_constraint.link_name = 'grasp_frame'
        
        bounding_volume = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        tolerance = position_tolerance if position_tolerance else self.position_tolerance
        sphere.dimensions = [tolerance]
        bounding_volume.primitives = [sphere]
        bounding_volume.primitive_poses = [target_pose.pose]
        pos_constraint.constraint_region = bounding_volume
        pos_constraint.weight = 1.0
        
        constraints.position_constraints = [pos_constraint]
        goal.request.goal_constraints = [constraints]
        
        # Plan only or plan+execute based on parameter
        goal.planning_options.plan_only = not self.execute_motion
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        
        self.get_logger().info(f'Planning to ({target_pose.pose.position.x:.3f}, '
                              f'{target_pose.pose.position.y:.3f}, '
                              f'{target_pose.pose.position.z:.3f})')
        if not self.execute_motion:
            self.get_logger().info('(plan only - execution disabled)')
        
        self.motion_in_progress = True
        future = self.move_action_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)
        
        return True
    
    def goal_response_callback(self, future):
        """Handle goal acceptance/rejection."""
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            self.motion_in_progress = False
            return
        
        self.current_goal_handle = goal_handle
        self.get_logger().info('Goal accepted, planning...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)
    
    def result_callback(self, future):
        """Handle planning/execution result."""
        result = future.result()
        error_code = result.result.error_code.val if result else -1
        
        self.motion_in_progress = False
        self.current_goal_handle = None
        
        if error_code != 1:
            self.get_logger().error(f'Motion failed: error code {error_code}')
            return
        
        self.get_logger().info('✓ Motion succeeded!')
        
        # Update joint positions from trajectory
        if self.execute_motion:
            trajectory = result.result.planned_trajectory
            if trajectory.joint_trajectory.points:
                final = trajectory.joint_trajectory.points[-1]
                names = trajectory.joint_trajectory.joint_names
                for i, name in enumerate(self.joint_names[:3]):
                    if name in names:
                        self.current_joint_positions[i] = final.positions[names.index(name)]
    
    def attach_object(self, object_id, link_name='grasp_frame'):
        """
        Attach collision object to robot link.
        
        Args:
            object_id: ID of collision object to attach
            link_name: Robot link to attach to
        """
        msg = AttachedCollisionObject()
        msg.object.id = object_id
        msg.link_name = link_name
        msg.object.operation = CollisionObject.ADD
        
        self.attached_collision_pub.publish(msg)
        self.get_logger().info(f'Attached {object_id} to {link_name}')
    
    def detach_object(self, object_id):
        """
        Detach collision object from robot.
        
        Args:
            object_id: ID of collision object to detach
        """
        msg = AttachedCollisionObject()
        msg.object.id = object_id
        msg.object.operation = CollisionObject.REMOVE
        
        self.attached_collision_pub.publish(msg)
        self.get_logger().info(f'Detached {object_id}')
    
    def send_gripper_command(self, angle_degrees):
        """
        Send gripper command.
        
        Args:
            angle_degrees: Gripper angle (0=closed, 155=open)
        """
        cmd = String()
        cmd.data = f'GRIPPER {angle_degrees}'
        self.motor_cmd_pub.publish(cmd)
        self.get_logger().info(f'Gripper command: {angle_degrees}°')


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
