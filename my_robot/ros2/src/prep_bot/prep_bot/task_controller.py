#!/usr/bin/env python3
"""
Task Controller (State Machine)

Orchestrates pick and place operations.

Full sequence:
1. Wait for cube_N + bin_X detections
2. Move to grasp pose
3. Close gripper
4. Lift
5. Move to place pose
6. Open gripper
"""
import os
import yaml
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from moveit_msgs.msg import Constraints, PositionConstraint, BoundingVolume, CollisionObject, JointConstraint
from moveit_msgs.action import MoveGroup
from moveit_msgs.srv import GetPlanningScene
from shape_msgs.msg import SolidPrimitive
from apriltag_msgs.msg import AprilTagDetectionArray
from tf2_ros import TransformListener, Buffer
from ament_index_python.packages import get_package_share_directory
from scripts.load_stl import load_stl_mesh
import rclpy.time
import rclpy.duration
from enum import Enum

class PickState(Enum):
    IDLE = 0
    WAITING_FOR_DETECTIONS = 1
    MOVING_TO_GRASP = 2
    CLOSING_GRIPPER = 3
    LIFTING = 4
    MOVING_TO_PLACE = 5
    OPENING_GRIPPER = 6
    MOVING_TO_READY = 7
    RECOVERING = 8


class TaskController(Node):
    def __init__(self):
        super().__init__('task_controller')
        
        # Declare parameters (no defaults - must be from config)
        self.declare_parameter('grasp_y_offset', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('grasp_tolerance', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('lift_height', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('place_height', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('place_tolerance', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('ready_joint_linear', 0.15)
        self.declare_parameter('ready_joint_shoulder', 0.7854)
        self.declare_parameter('ready_joint_elbow', 0.7854)
        
        # Validate required parameters
        required = ['grasp_y_offset', 'grasp_tolerance', 'lift_height', 'place_height', 'place_tolerance']
        missing = [p for p in required if not self.has_parameter(p) or 
                   self.get_parameter(p).value is None]
        if missing:
            self.get_logger().error(f'Required parameters missing: {", ".join(missing)}')
            raise RuntimeError('Missing required configuration')
        
        # Get parameter values
        self.grasp_y_offset = self.get_parameter('grasp_y_offset').value
        self.grasp_tolerance = self.get_parameter('grasp_tolerance').value
        self.lift_height = self.get_parameter('lift_height').value
        self.place_height = self.get_parameter('place_height').value
        self.place_tolerance = self.get_parameter('place_tolerance').value
        self.ready_joints = {
            'linear_joint': self.get_parameter('ready_joint_linear').value,
            'shoulder_joint': self.get_parameter('ready_joint_shoulder').value,
            'elbow_joint': self.get_parameter('ready_joint_elbow').value
        }
        
        # Target bin for placing
        self.target_bin_id = 11
        
        # Load bin mesh for center calculation
        pkg_share = get_package_share_directory('prep_bot')
        bin_mesh_path = os.path.join(pkg_share, 'meshes', 'bin.stl')
        self.bin_mesh_center = np.array([0.0, 0.0, 0.0])
        
        if os.path.exists(bin_mesh_path):
            bin_mesh = load_stl_mesh(bin_mesh_path)
            vertices = np.array([[v.x, v.y, v.z] for v in bin_mesh.vertices])
            self.bin_mesh_center = np.mean(vertices, axis=0)
            self.get_logger().debug(f'Bin mesh center: ({self.bin_mesh_center[0]:.3f}, '
                                  f'{self.bin_mesh_center[1]:.3f}, {self.bin_mesh_center[2]:.3f})')
        else:
            self.get_logger().warn(f'Bin mesh not found: {bin_mesh_path}')

        # Load object config for tag offsets
        pkg_share = get_package_share_directory('prep_bot')
        config_path = os.path.join(pkg_share, 'config', 'tags.yaml')
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
            ros_params = config.get('/**', {}).get('ros__parameters', {})
            self.objects = ros_params.get('objects', {})
        
        # Action client for motion_controller
        self.motion_client = ActionClient(
            self, MoveGroup, 'motion_controller/move_group'
        )
        
        # Service client for planning scene queries
        self.get_planning_scene_client = self.create_client(
            GetPlanningScene, '/get_planning_scene'
        )
        
        # Service client for gripper control
        self.gripper_client = self.create_client(SetBool, '/hardware_interface/gripper')
        
        # Track current linear position from joint states
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        joint_state_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.current_linear_position = 0.0
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, joint_state_qos)
        
        # Publishers
        self.collision_pub = self.create_publisher(
            CollisionObject, '/collision_object', 10
        )
        self.scene_cmd_pub = self.create_publisher(String, '/scene_manager/command', 10)
        self.move_target_pub = self.create_publisher(PoseStamped, '/scene_manager/move_target', 10)
        self.pick_status_pub = self.create_publisher(String, '/pick_status', 10)
        
        # TF for detections
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscribe to pick commands and detections
        self.create_subscription(String, '/pick_command', self.pick_command_callback, 10)
        self.create_subscription(AprilTagDetectionArray, '/detections', self.detections_callback, 10)
        
        # State
        self.state = PickState.IDLE
        self.target_cube_id = None
        self.cube_pose = None
        self.bin_pose = None
        self.detected_tags = {}
        self.picking_in_progress = False  # Stop cube updates during pick
        
        self.get_logger().info('Task Controller Ready')
        self.get_logger().info('Commands: PICK <cube_id>, HOME')
    
    def joint_state_callback(self, msg):
        """Track current linear joint position."""
        if 'linear_joint' in msg.name:
            idx = msg.name.index('linear_joint')
            if idx < len(msg.position):
                self.current_linear_position = msg.position[idx]
    
    def detections_callback(self, msg):
        """Track detected AprilTags."""
        # Skip updates during picking
        if self.picking_in_progress:
            return
            
        for detection in msg.detections:
            tag_id = detection.id
            tag_frame = f'{detection.family}:{tag_id}'
            
            try:
                transform = self.tf_buffer.lookup_transform(
                    'base_link', tag_frame, rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.1)
                )
                
                pose = PoseStamped()
                pose.header.frame_id = 'base_link'
                pose.pose.position.x = transform.transform.translation.x
                pose.pose.position.y = transform.transform.translation.y
                pose.pose.position.z = transform.transform.translation.z
                pose.pose.orientation = transform.transform.rotation
                
                self.detected_tags[tag_id] = pose
            except Exception:
                pass
    
    def pick_command_callback(self, msg):
        """Handle pick commands."""
        cmd = msg.data.strip().upper()
        
        # HOME command can be sent anytime when IDLE
        if cmd == 'HOME':
            if self.state != PickState.IDLE:
                self.get_logger().warn(f'Busy in state {self.state.name}, ignoring HOME')
                return
            self.execute_full_home()
            return
        
        if self.state != PickState.IDLE:
            self.get_logger().warn(f'Busy in state {self.state.name}, ignoring command')
            return
        
        if cmd.startswith('PICK '):
            parts = cmd.split()
            if len(parts) != 2:
                self.get_logger().error('Usage: PICK <cube_id>')
                return
            
            try:
                cube_id = int(parts[1])
                self.start_pick_sequence(cube_id)
            except ValueError:
                self.get_logger().error('Invalid cube_id')
    
    def start_pick_sequence(self, cube_id):
        """Start pick and place sequence."""
        self.target_cube_id = cube_id
        self.cube_pose = None
        self.bin_pose = None
        self.picking_in_progress = True
        
        # Ensure gripper is open before starting
        self._call_gripper_service(False)  # False = open
        
        self.get_logger().info(f'Starting pick sequence: cube_{cube_id} → bin_{self.target_bin_id}')
        self.state = PickState.WAITING_FOR_DETECTIONS
        self.check_detections()
    
    def check_detections(self):
        """Check if we have required detections (cube and bin)."""
        if self.target_cube_id in self.detected_tags:
            self.cube_pose = self.detected_tags[self.target_cube_id]
            # Adjust Z to cube base (tag is on top)
            self.cube_pose.pose.position.z -= 0.0175
            self.cube_pose.pose.orientation.w = 1.0
        
        if self.target_bin_id in self.detected_tags:
            raw_pose = self.detected_tags[self.target_bin_id]
            self.bin_pose = PoseStamped()
            self.bin_pose.header = raw_pose.header
            self.bin_pose.pose = Pose()
            
            # Apply tag offset from config
            obj_config = self.objects.get(self.target_bin_id, {})
            self.bin_pose.pose.position.x = raw_pose.pose.position.x + obj_config.get('offset_x', 0.0)
            self.bin_pose.pose.position.y = raw_pose.pose.position.y + obj_config.get('offset_y', 0.0)
            self.bin_pose.pose.position.z = raw_pose.pose.position.z + obj_config.get('offset_z', 0.0)
            self.bin_pose.pose.orientation = raw_pose.pose.orientation
        
        if self.cube_pose and self.bin_pose:
            self.get_logger().debug(f'✓ Cube detected at ({self.cube_pose.pose.position.x:.3f}, '
                                  f'{self.cube_pose.pose.position.y:.3f}, {self.cube_pose.pose.position.z:.3f})')
            self.get_logger().debug(f'✓ Bin detected at ({self.bin_pose.pose.position.x:.3f}, '
                                  f'{self.bin_pose.pose.position.y:.3f}, {self.bin_pose.pose.position.z:.3f})')
            self.execute_grasp()
        else:
            missing = []
            if not self.cube_pose:
                missing.append(f'cube_{self.target_cube_id}')
            if not self.bin_pose:
                missing.append(f'bin_{self.target_bin_id}')
            self.get_logger().warn(f'Waiting for detections: {", ".join(missing)}')
    
    def execute_grasp(self):
        """Move to grasp pose."""
        self.state = PickState.MOVING_TO_GRASP
        self._grasp_attempts = 0
        self._max_grasp_attempts = 3
        
        self._attempt_grasp()
    
    def _attempt_grasp(self):
        """Attempt to move to grasp pose (with retry support)."""
        self._grasp_attempts += 1
        
        self._pause_scene_updates()
        
        grasp_pose = PoseStamped()
        grasp_pose.header.frame_id = 'base_link'
        grasp_pose.header.stamp = self.get_clock().now().to_msg()
        grasp_pose.pose.position.x = self.cube_pose.pose.position.x
        grasp_pose.pose.position.y = self.cube_pose.pose.position.y + self.grasp_y_offset
        grasp_pose.pose.position.z = self.cube_pose.pose.position.z
        grasp_pose.pose.orientation.w = 1.0
        
        self.get_logger().debug(f'→ Moving to grasp pose (attempt {self._grasp_attempts}/{self._max_grasp_attempts})')
        self.send_motion_goal(grasp_pose, tolerance=self.grasp_tolerance)
    
    # =========================================================================
    # Gripper close (after reaching grasp pose)
    # =========================================================================
    
    def execute_close_gripper(self):
        """Close gripper to grasp the cube."""
        self.state = PickState.CLOSING_GRIPPER
        self.get_logger().debug('→ Closing gripper')
        
        self._call_gripper_service(True)  # True = close
        
        # For now, use a one-shot timer to wait then finish (TODO: implement gripper feedback)
        self._gripper_timer = self.create_timer(1.5, self._gripper_close_done)
    
    def _gripper_close_done(self):
        """Called after gripper close delay."""
        self._gripper_timer.cancel()
        
        self.get_logger().debug('✓ Gripper closed')
        
        # Attach cube to gripper via scene_manager
        cube_name = f'cube_{self.target_cube_id}'
        attach_cmd = String()
        attach_cmd.data = f'ATTACH {cube_name} grasp_frame'
        self.scene_cmd_pub.publish(attach_cmd)
        self.get_logger().debug(f'Attaching {cube_name} to gripper...')
        
        # Wait for MoveIt to have the attached object
        self.wait_for_attached_object(cube_name, callback=self.execute_lift)
    
    def wait_for_attached_object(self, object_name, callback, timeout=5.0, poll_interval=0.1):
        """Poll planning scene until attached object appears, then call callback."""
        self._wait_object_name = object_name
        self._wait_callback = callback
        self._wait_timeout = timeout
        self._wait_start = self.get_clock().now()
        self._wait_poll_interval = poll_interval
        self._wait_completed = False
        self._wait_timer = self.create_timer(poll_interval, self._check_attached_object)
    
    def _check_attached_object(self):
        """Check if attached object is in planning scene."""
        if self._wait_completed:
            return
        
        # Check timeout
        elapsed = (self.get_clock().now() - self._wait_start).nanoseconds / 1e9
        if elapsed > self._wait_timeout:
            self._wait_timer.cancel()
            self.get_logger().error(f'Timeout waiting for {self._wait_object_name} in planning scene')
            self.cleanup_on_failure()
            return
        
        # Query planning scene
        if not self.get_planning_scene_client.service_is_ready():
            return  # Service not ready, try again
        
        request = GetPlanningScene.Request()
        request.components.components = request.components.ROBOT_STATE_ATTACHED_OBJECTS
        
        future = self.get_planning_scene_client.call_async(request)
        future.add_done_callback(self._planning_scene_response)
    
    def _planning_scene_response(self, future):
        """Handle planning scene response."""
        # Skip if already completed (another callback beat us)
        if self._wait_completed:
            return
        
        try:
            response = future.result()
            attached_objects = response.scene.robot_state.attached_collision_objects
            
            for obj in attached_objects:
                if obj.object.id == self._wait_object_name:
                    self._wait_completed = True  # Mark completed before callback
                    self._wait_timer.cancel()
                    self.get_logger().info(f'✓ {self._wait_object_name} attached in planning scene')
                    self._wait_callback()
                    return
            
            # Object not found yet, timer will retry
        except Exception as e:
            self.get_logger().warn(f'Planning scene query failed: {e}')
    
    # =========================================================================
    # Lift (after gripper close)
    # =========================================================================
    
    def execute_lift(self):
        """Lift object straight up."""
        self.state = PickState.LIFTING
        
        self._pause_scene_updates()
        
        lift_pose = PoseStamped()
        lift_pose.header.frame_id = 'base_link'
        lift_pose.header.stamp = self.get_clock().now().to_msg()
        lift_pose.pose.position.x = self.cube_pose.pose.position.x
        lift_pose.pose.position.y = self.cube_pose.pose.position.y + self.grasp_y_offset
        lift_pose.pose.position.z = self.cube_pose.pose.position.z + self.lift_height
        lift_pose.pose.orientation.w = 1.0
        
        self.get_logger().debug(f'→ Lifting {self.lift_height}m')
        self.send_motion_goal(lift_pose, tolerance=0.02)
    
    def execute_place(self):
        """Move to place pose above bin center."""
        self.state = PickState.MOVING_TO_PLACE
        
        self._pause_scene_updates()
        
        # bin_pose already has tag offset applied
        # Add mesh center offset to get actual bin center
        place_pose = PoseStamped()
        place_pose.header.frame_id = 'base_link'
        place_pose.header.stamp = self.get_clock().now().to_msg()
        place_pose.pose.position.x = self.bin_pose.pose.position.x + self.bin_mesh_center[0]
        place_pose.pose.position.y = self.bin_pose.pose.position.y + self.bin_mesh_center[1] + self.grasp_y_offset
        place_pose.pose.position.z = self.bin_pose.pose.position.z + self.bin_mesh_center[2] + self.place_height
        place_pose.pose.orientation.w = 1.0
        
        self.get_logger().debug(f'→ Moving to place pose at ({place_pose.pose.position.x:.3f}, '
                              f'{place_pose.pose.position.y:.3f}, {place_pose.pose.position.z:.3f})')
        self.send_motion_goal(place_pose, tolerance=self.place_tolerance)
    
    def execute_open_gripper(self):
        """Open gripper to release the cube."""
        self.state = PickState.OPENING_GRIPPER
        self.get_logger().debug('→ Opening gripper')
        
        self._call_gripper_service(False)  # False = open
        
        # Wait for gripper to open, then finish
        self._gripper_open_timer = self.create_timer(1.0, self._gripper_open_done)
    
    def _gripper_open_done(self):
        """Called after gripper open delay."""
        self._gripper_open_timer.cancel()
        
        self.get_logger().debug('✓ Gripper opened')
        
        # Remove cube from scene
        cube_name = f'cube_{self.target_cube_id}'
        remove_cmd = String()
        remove_cmd.data = f'REMOVE_PERMANENT {cube_name}'
        self.scene_cmd_pub.publish(remove_cmd)
        self.get_logger().debug(f'Published REMOVE_PERMANENT {cube_name}')
        
        self.execute_quick_ready()
    
    # =========================================================================
    # Return to ready positions
    # =========================================================================

    def execute_quick_ready(self):
        """Move shoulder/elbow to ready position, keep linear axis unchanged.
        
        This is faster for consecutive picks. Automation-service sends explicit 
        /robot/home after order completes to return to full ready position.
        """
        self.state = PickState.MOVING_TO_READY
        self._pause_scene_updates()
        
        # Set all three joints: current linear + ready shoulder/elbow
        quick_ready_joints = {
            'linear_joint': self.current_linear_position,
            'shoulder_joint': self.ready_joints['shoulder_joint'],
            'elbow_joint': self.ready_joints['elbow_joint']
        }
        
        self.get_logger().debug(f'→ Moving to quick ready (linear={self.current_linear_position:.3f})')
        self.send_joint_goal(quick_ready_joints, tolerance=0.05)
    
    def execute_full_home(self):
        """Move to full ready position including linear retract.
        
        Called via /robot/home endpoint after order completes.
        """
        if self.state != PickState.IDLE:
            self.get_logger().warn(f'Cannot go home while in state {self.state.name}')
            return False
        
        self.state = PickState.MOVING_TO_READY
        self._is_full_home = True  # Flag to publish IDLE instead of SUCCESS
        self._pause_scene_updates()
        
        self.get_logger().debug('→ Returning to full ready position')
        self.send_joint_goal(self.ready_joints, tolerance=0.05)
        return True

    def finish_sequence(self):
        """Complete the pick and place sequence."""
        self.get_logger().info('✓ Pick and place sequence complete!')
        
        # Resume scene updates
        self._resume_scene_updates()
        
        # Publish appropriate status
        status_msg = String()
        if getattr(self, '_is_full_home', False):
            # HOME command completed - publish IDLE
            status_msg.data = 'IDLE'
            self._is_full_home = False
        else:
            # Pick sequence completed - publish SUCCESS
            status_msg.data = f'SUCCESS {self.target_cube_id}'
        self.pick_status_pub.publish(status_msg)
        
        self.state = PickState.IDLE
        self.target_cube_id = None
        self.cube_pose = None
        self.bin_pose = None
        self.picking_in_progress = False
    
    def cleanup_on_failure(self):
        """Clean up state when pick sequence fails."""
        self.get_logger().error('✗ Pick sequence failed')
        
        # Resume scene updates
        self._resume_scene_updates()
        
        # Publish failure status
        status_msg = String()
        status_msg.data = f'FAILED {self.target_cube_id if self.target_cube_id else 0}'
        self.pick_status_pub.publish(status_msg)
        
        # Save current state before recovery changes it
        failed_state = self.state
        
        # Start recovery sequence
        self._start_recovery(failed_state)
    
    def _start_recovery(self, failed_state=None):
        """Recovery sequence: detach cube, open gripper, return to ready."""
        self.state = PickState.RECOVERING  # Set immediately to prevent new commands
        self.get_logger().info('Starting recovery sequence...')
        
        # Detach cube if we got past gripper close (cube was attached)
        states_with_attached_cube = [PickState.LIFTING, PickState.MOVING_TO_PLACE, PickState.OPENING_GRIPPER]
        if self.target_cube_id and failed_state in states_with_attached_cube:
            cube_name = f'cube_{self.target_cube_id}'
            remove_cmd = String()
            remove_cmd.data = f'REMOVE {cube_name}'
            self.scene_cmd_pub.publish(remove_cmd)
            self.get_logger().info(f'Detached {cube_name}')
        
        # Open gripper
        self._call_gripper_service(False)  # False = open
        
        # Wait for gripper, then return home
        self._recovery_timer = self.create_timer(1.0, self._recovery_return_home)
    
    def _recovery_return_home(self):
        """Return to ready position during recovery."""
        self._recovery_timer.cancel()
        
        self.get_logger().info('Recovery: returning to ready position')
        
        # Pause scene for motion
        self._pause_scene_updates()
        
        self.send_joint_goal(self.ready_joints, tolerance=0.05)
    
    def _finish_recovery(self):
        """Complete recovery and reset state."""
        self.get_logger().info('Recovery complete')
        
        self._resume_scene_updates()
        
        self.state = PickState.IDLE
        self.target_cube_id = None
        self.cube_pose = None
        self.bin_pose = None
        self.picking_in_progress = False
        
        # Publish IDLE status so Flask endpoint knows robot is ready
        status_msg = String()
        status_msg.data = 'IDLE'
        self.pick_status_pub.publish(status_msg)
    
    def _pause_scene_updates(self):
        """Pause scene_manager collision object updates during motion planning."""
        cmd = String()
        cmd.data = 'PAUSE_ALL'
        self.scene_cmd_pub.publish(cmd)
        self.get_logger().debug('Published PAUSE_ALL to scene_manager')
    
    def _resume_scene_updates(self):
        """Resume scene_manager collision object updates after motion completes."""
        cmd = String()
        cmd.data = 'RESUME_ALL'
        self.scene_cmd_pub.publish(cmd)
        self.get_logger().debug('Published RESUME_ALL to scene_manager')
    
    def send_motion_goal(self, target_pose, tolerance=0.005, remove_objects=None, use_cylinder=False, cylinder_height=0.1):
        """Send motion goal to motion_controller action server.
        
        Args:
            target_pose: Target pose for grasp_frame
            tolerance: Position tolerance (sphere radius or cylinder radius)
            remove_objects: List of object IDs to remove from planning scene for this motion
            use_cylinder: If True, use cylinder constraint (for Z flexibility)
            cylinder_height: Height of cylinder constraint (Z tolerance)
        """
        # Visualization
        viz_pose = PoseStamped()
        viz_pose.header.frame_id = f'base_link:{tolerance}'
        viz_pose.header.stamp = self.get_clock().now().to_msg()
        viz_pose.pose = target_pose.pose
        self.move_target_pub.publish(viz_pose)
        
        if not self.motion_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('motion_controller action server not available!')
            self.cleanup_on_failure()
            return
        
        # Build MoveGroup goal
        goal = MoveGroup.Goal()
        goal.request.group_name = 'robot_arm'
        
        # Position constraint
        constraints = Constraints()
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = 'base_link'
        pos_constraint.link_name = 'grasp_frame'
        
        bounding_volume = BoundingVolume()
        if use_cylinder:
            shape = SolidPrimitive()
            shape.type = SolidPrimitive.CYLINDER
            shape.dimensions = [cylinder_height, tolerance]  # [height, radius]
        else:
            shape = SolidPrimitive()
            shape.type = SolidPrimitive.SPHERE
            shape.dimensions = [tolerance]
        bounding_volume.primitives = [shape]
        bounding_volume.primitive_poses = [target_pose.pose]
        pos_constraint.constraint_region = bounding_volume
        pos_constraint.weight = 1.0
        
        constraints.position_constraints = [pos_constraint]
        goal.request.goal_constraints = [constraints]
        
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        
        # Add objects to remove from planning scene
        if remove_objects:
            for obj_id in remove_objects:
                remove_obj = CollisionObject()
                remove_obj.id = obj_id
                remove_obj.operation = CollisionObject.REMOVE
                goal.planning_options.planning_scene_diff.world.collision_objects.append(remove_obj)
            self.get_logger().info(f'Planning with {len(remove_objects)} objects removed from scene')
        
        send_goal_future = self.motion_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self.motion_goal_response_callback)
    
    def send_joint_goal(self, joint_targets: dict, tolerance=0.01):
        """Send joint goal to motion_controller action server.
        
        Args:
            joint_targets: Dict of joint_name -> target_value (radians or meters)
            tolerance: Joint tolerance
        """
        if not self.motion_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('motion_controller action server not available!')
            self.cleanup_on_failure()
            return
        
        # Build MoveGroup goal with joint constraints
        goal = MoveGroup.Goal()
        goal.request.group_name = 'robot_arm'
        
        constraints = Constraints()
        for joint_name, target_value in joint_targets.items():
            jc = JointConstraint()
            jc.joint_name = joint_name
            jc.position = target_value
            jc.tolerance_above = tolerance
            jc.tolerance_below = tolerance
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        
        goal.request.goal_constraints = [constraints]
        
        # Use planning scene diff
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        
        send_goal_future = self.motion_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self.motion_goal_response_callback)
    
    def motion_goal_response_callback(self, future):
        """Handle goal acceptance/rejection."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Motion goal rejected!')
            self.cleanup_on_failure()
            return
        
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.motion_result_callback)
    
    def motion_result_callback(self, future):
        """Handle motion result and advance state machine."""
        try:
            result = future.result().result
            if result.error_code.val != 1:
                self.get_logger().error(f'✗ Motion failed (code: {result.error_code.val})')
                
                # Retry logic for grasp motion (planning failures)
                if self.state == PickState.MOVING_TO_GRASP:
                    if self._grasp_attempts < self._max_grasp_attempts:
                        self.get_logger().info(f'Retrying grasp motion...')
                        # Resume scene briefly to let it update, then retry
                        self._resume_scene_updates()
                        # Small delay before retry
                        self._retry_timer = self.create_timer(0.5, self._retry_grasp)
                        return
                    else:
                        self.get_logger().error(f'✗ Grasp failed after {self._max_grasp_attempts} attempts')
                
                # If we're in recovery, just finish (don't recurse)
                if self.state == PickState.RECOVERING:
                    self._finish_recovery()
                else:
                    self.cleanup_on_failure()
                return
            
            self.get_logger().info('✓ Motion completed')
            
            # After reaching grasp pose, close gripper
            if self.state == PickState.MOVING_TO_GRASP:
                self.execute_close_gripper()
            elif self.state == PickState.LIFTING:
                self.execute_place()
            elif self.state == PickState.MOVING_TO_PLACE:
                self.execute_open_gripper()
            elif self.state == PickState.MOVING_TO_READY:
                self.finish_sequence()
            elif self.state == PickState.RECOVERING:
                self._finish_recovery()
        except Exception as e:
            self.get_logger().error(f'Exception in motion_result_callback: {e}')
            if self.state != PickState.RECOVERING:
                self.cleanup_on_failure()
            else:
                self._finish_recovery()
    
    def _retry_grasp(self):
        """Retry grasp motion after a brief delay."""
        self._retry_timer.cancel()
        self._attempt_grasp()
    
    def _call_gripper_service(self, close: bool):
        """Call gripper service (non-blocking).
        
        Args:
            close: True to close gripper, False to open
        """
        if self.gripper_client.wait_for_service(timeout_sec=0.5):
            req = SetBool.Request()
            req.data = close
            future = self.gripper_client.call_async(req)
        else:
            self.get_logger().warn('Gripper service not available')


def main(args=None):
    rclpy.init(args=args)
    node = TaskController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
