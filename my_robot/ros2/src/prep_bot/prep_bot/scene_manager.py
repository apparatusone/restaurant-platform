#!/usr/bin/env python3
"""
Scene Manager

Detects objects and bins via AprilTag, publishes collision objects to MoveIt planning scene.
Provides object pose information for motion planning.
Visualizes move targets and robot workspace.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from apriltag_msgs.msg import AprilTagDetectionArray
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
from shape_msgs.msg import SolidPrimitive
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from std_srvs.srv import Trigger, SetBool
from tf2_ros import TransformListener, Buffer
from ament_index_python.packages import get_package_share_directory
from scipy.spatial.transform import Rotation
import rclpy.time
import rclpy.duration
import yaml
import os

from scripts.load_stl import load_stl_mesh


class SceneManagerNode(Node):
    def __init__(self):
        super().__init__('scene_manager')

        self.declare_parameter('workspace_radius', 0.29)
        self.declare_parameter('shoulder_z', 0.211)
        self.declare_parameter('linear_home_x', -0.4061)
        self.declare_parameter('linear_max_x', 0.4062)
        self.declare_parameter('default_cube_size', 0.035)
        
        # Load object config
        pkg_share = get_package_share_directory('prep_bot')
        config_path = os.path.join(pkg_share, 'config', 'tags.yaml')
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
            ros_params = config.get('/**', {}).get('ros__parameters', {})
            self.objects = ros_params.get('objects', {})
        
        self.get_logger().info(f'Loaded {len(self.objects)} object definitions')
        
        # Load bin mesh
        mesh_path = os.path.join(pkg_share, 'meshes', 'bin.stl')
        if os.path.exists(mesh_path):
            self.bin_mesh = load_stl_mesh(mesh_path)
            self.get_logger().debug(f'Loaded bin mesh: {len(self.bin_mesh.triangles)} triangles')
        else:
            self.bin_mesh = None
            self.get_logger().warn(f'Bin mesh not found: {mesh_path}')
        
        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Publishers
        self.collision_pub = self.create_publisher(CollisionObject, '/collision_object', 10)
        self.attached_collision_pub = self.create_publisher(AttachedCollisionObject, '/attached_collision_object', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/scene_manager/markers', 10)
        
        # Services
        self.create_service(Trigger, '/scene_manager/get_objects', self.get_objects_callback)
        self.create_service(SetBool, '/scene_manager/pause_updates', self.pause_updates_callback)
        
        # Subscribers
        self.create_subscription(AprilTagDetectionArray, '/detections', self.tag_callback, 10)
        
        # Subscribe to scene commands from task_controller (PAUSE, RESUME, ATTACH, REMOVE)
        from std_msgs.msg import String
        self.create_subscription(String, '/scene_manager/command', self.scene_command_callback, 10)
        
        # Subscribe to move target visualization
        self.create_subscription(PoseStamped, '/scene_manager/move_target', self.move_target_callback, 10)
        
        # Detected objects (keyed by name)
        self.detected_objects = {}
        
        # Paused objects
        self.paused_objects = set()
        
        # Attached objects (currently attached to robot)
        self.attached_objects = set()
        
        # Permanently removed objects (never re-add even if tag seen again)
        self.permanently_removed = set()
        
        # when True, no collision objects are updated
        self._updates_paused = False
        
        # Current move target
        self.current_target = None
        self.current_tolerance = 0.05  # Default 50mm
        
        # Workspace cylinder parameters        
        self.workspace_radius = self.get_parameter('workspace_radius').value
        self.shoulder_z = self.get_parameter('shoulder_z').value
        self.linear_home_x = self.get_parameter('linear_home_x').value
        self.linear_max_x = self.get_parameter('linear_max_x').value
        self.default_cube_size = self.get_parameter('default_cube_size').value
        
        # Publish workspace visualization
        self.create_timer(2.0, self.publish_markers)
        
        self.get_logger().info('Scene Manager Ready\n')
    
    def publish_collision_object(self, obj_name, obj_config, pose):
        """Publish collision object to MoveIt planning scene"""
        # Skip objects without collision shapes
        if 'collision_shape' not in obj_config:
            return
        
        obj = CollisionObject()
        obj.header.stamp = self.get_clock().now().to_msg()
        obj.header.frame_id = 'base_link'
        obj.id = obj_name
        
        # Ignore any rotation in x/y for bins
        if obj_config.get('collision_shape') == 'mesh' and self.bin_mesh:
            r = Rotation.from_quat([
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w
            ])
            euler = r.as_euler('xyz')
            r_flat = Rotation.from_euler('xyz', [0, 0, euler[2]])
            quat_flat = r_flat.as_quat()
            
            pose.pose.orientation.x = quat_flat[0]
            pose.pose.orientation.y = quat_flat[1]
            pose.pose.orientation.z = quat_flat[2]
            pose.pose.orientation.w = quat_flat[3]
            
            obj.meshes = [self.bin_mesh]
            obj.mesh_poses = [pose.pose]
        
        elif obj_config.get('collision_shape') == 'box':
            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = obj_config.get('dimensions', [self.default_cube_size] * 3)
            obj.primitives = [box]
            obj.primitive_poses = [pose.pose]
        
        obj.operation = CollisionObject.ADD
        self.collision_pub.publish(obj)
    
    def remove_collision_object(self, obj_name):
        """Remove a collision object from the planning scene."""
        obj = CollisionObject()
        obj.header.stamp = self.get_clock().now().to_msg()
        obj.header.frame_id = 'base_link'
        obj.id = obj_name
        obj.operation = CollisionObject.REMOVE
        self.collision_pub.publish(obj)
    
    def attach_object_to_link(self, obj_name, link_name):
        """Attach a collision object (cube) to a robot link."""
        self.remove_collision_object(obj_name)
        
        # Find the tag_id from obj_name (e.g., cube_5 -> 5)
        tag_id = None
        if obj_name.startswith('cube_'):
            try:
                tag_id = int(obj_name.split('_')[1])
            except (IndexError, ValueError):
                pass
        
        obj_config = self.objects.get(tag_id, {}) if tag_id else {}
        dimensions = obj_config.get('dimensions', [self.default_cube_size] * 3)
        
        # Create attached collision object
        msg = AttachedCollisionObject()
        msg.link_name = link_name
        msg.object.header.frame_id = link_name
        msg.object.header.stamp = self.get_clock().now().to_msg()
        msg.object.id = obj_name
        msg.object.operation = CollisionObject.ADD
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = dimensions
        msg.object.primitives = [box]
        
        # Pose relative to link (centered)
        from geometry_msgs.msg import Pose
        pose = Pose()
        pose.orientation.w = 1.0
        msg.object.primitive_poses = [pose]
        
        # Touch links - allow contact with gripper parts
        msg.touch_links = ['grasp_frame', 'gripper_left', 'gripper_right', 'gripper_body', 'elbow_link']
        
        self.attached_collision_pub.publish(msg)
        self.paused_objects.add(obj_name)  # Don't update while attached
        self.attached_objects.add(obj_name)  # Track attachment
        self.get_logger().info(f'Attached {obj_name} to {link_name}')
    
    def move_target_callback(self, msg):
        """Receive move target for visualization."""
        self.current_target = msg
        # Parse tolerance from frame_id if provided (format: "base_link:0.05")
        if ':' in msg.header.frame_id:
            parts = msg.header.frame_id.split(':')
            try:
                self.current_tolerance = float(parts[1])
            except ValueError:
                pass
        self.publish_markers()
    
    def get_objects_callback(self, request, response):
        """Service callback to return detected objects as JSON string"""
        import json
        objects_list = []
        for obj_name, pose in self.detected_objects.items():
            objects_list.append({
                'name': obj_name,
                'x': pose.pose.position.x,
                'y': pose.pose.position.y,
                'z': pose.pose.position.z
            })
        response.success = True
        response.message = json.dumps(objects_list)
        return response
    
    def pause_updates_callback(self, request, response):
        """Service callback to pause/resume all collision object updates"""
        self._updates_paused = request.data
        response.success = True
        response.message = 'Updates paused' if request.data else 'Updates resumed'
        self.get_logger().info(response.message)
        return response
    
    def scene_command_callback(self, msg):
        """Handle scene commands: PAUSE, RESUME, ATTACH, REMOVE, etc."""
        cmd = msg.data.strip()
        if cmd.startswith('PAUSE '):
            obj_name = cmd[6:]
            self.paused_objects.add(obj_name)
            self.remove_collision_object(obj_name)
            self.get_logger().debug(f'Paused and removed {obj_name}')
        elif cmd.startswith('RESUME '):
            obj_name = cmd[7:]
            self.paused_objects.discard(obj_name)
            self.get_logger().debug(f'Resumed updates for {obj_name}')
        elif cmd.startswith('ATTACH '):
            parts = cmd.split()
            if len(parts) >= 3:
                self.attach_object_to_link(parts[1], parts[2])
            else:
                self.get_logger().debug('ATTACH requires: ATTACH <obj_name> <link_name>')
        elif cmd.startswith('REMOVE '):
            obj_name = cmd[7:]
            self._remove_object(obj_name, permanent=False)
        elif cmd.startswith('REMOVE_PERMANENT '):
            obj_name = cmd[17:]
            self._remove_object(obj_name, permanent=True)
        elif cmd == 'PAUSE_ALL':
            self._updates_paused = True
            self.get_logger().debug('Paused all collision object updates')
        elif cmd == 'RESUME_ALL':
            self._updates_paused = False
            self.get_logger().debug('Resumed collision object updates')
    
    def _remove_object(self, obj_name, permanent=False):
        """Remove an object from the scene."""
        self.paused_objects.discard(obj_name)
        if obj_name in self.detected_objects:
            del self.detected_objects[obj_name]
        
        if permanent:
            self.permanently_removed.add(obj_name)
        
        self.remove_collision_object(obj_name)
        
        # Only remove as attached object if it was actually attached
        if obj_name in self.attached_objects:
            attached_msg = AttachedCollisionObject()
            attached_msg.link_name = 'grasp_frame'
            attached_msg.object.id = obj_name
            attached_msg.object.header.frame_id = 'base_link'
            attached_msg.object.operation = CollisionObject.REMOVE
            self.attached_collision_pub.publish(attached_msg)
            self.attached_objects.discard(obj_name)
        
        if permanent:
            self.remove_collision_object(obj_name)
            self.get_logger().debug(f'Permanently removed {obj_name}')
        else:
            self.get_logger().debug(f'Removed {obj_name}')
    
    def publish_markers(self):
        """Publish visualization markers for workspace and move target."""
        # Skip if no subscribers (RViz not open or not subscribed)
        if self.marker_pub.get_subscription_count() == 0:
            return

        markers = MarkerArray()
        
        # Workspace cylinder (robot reach envelope)
        workspace_marker = Marker()
        workspace_marker.header.frame_id = 'base_link'
        workspace_marker.header.stamp = self.get_clock().now().to_msg()
        workspace_marker.ns = 'workspace'
        workspace_marker.id = 0
        workspace_marker.type = Marker.CYLINDER
        workspace_marker.action = Marker.ADD
        
        # Position at center of linear travel, at shoulder axis
        workspace_marker.pose.position.x = (self.linear_home_x + self.linear_max_x) / 2
        workspace_marker.pose.position.y = 0.0
        workspace_marker.pose.position.z = self.shoulder_z
        
        # Rotate 90° around Y to align cylinder with X axis
        from scipy.spatial.transform import Rotation
        r = Rotation.from_euler('y', 90, degrees=True)
        q = r.as_quat()
        workspace_marker.pose.orientation.x = q[0]
        workspace_marker.pose.orientation.y = q[1]
        workspace_marker.pose.orientation.z = q[2]
        workspace_marker.pose.orientation.w = q[3]
        
        # Scale: x/y = diameter, z = height (length along X axis)
        workspace_marker.scale.x = self.workspace_radius * 2
        workspace_marker.scale.y = self.workspace_radius * 2
        workspace_marker.scale.z = self.linear_max_x - self.linear_home_x
        
        workspace_marker.color = ColorRGBA(r=0.5, g=0.5, b=1.0, a=0.2)
        markers.markers.append(workspace_marker)
        
        # Move target sphere (tolerance visualization)
        if self.current_target:
            target_marker = Marker()
            target_marker.header.frame_id = 'base_link'
            target_marker.header.stamp = self.get_clock().now().to_msg()
            target_marker.ns = 'move_target'
            target_marker.id = 0
            target_marker.type = Marker.SPHERE
            target_marker.action = Marker.ADD
            target_marker.pose = self.current_target.pose
            target_marker.scale.x = self.current_tolerance * 2
            target_marker.scale.y = self.current_tolerance * 2
            target_marker.scale.z = self.current_tolerance * 2
            target_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.3)
            markers.markers.append(target_marker)
            
            # Target center point
            center_marker = Marker()
            center_marker.header.frame_id = 'base_link'
            center_marker.header.stamp = self.get_clock().now().to_msg()
            center_marker.ns = 'move_target'
            center_marker.id = 1
            center_marker.type = Marker.SPHERE
            center_marker.action = Marker.ADD
            center_marker.pose = self.current_target.pose
            center_marker.scale.x = 0.01
            center_marker.scale.y = 0.01
            center_marker.scale.z = 0.01
            center_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
            markers.markers.append(center_marker)
        
        self.marker_pub.publish(markers)
    
    def tag_callback(self, msg):
        if not msg.detections:
            return
        
        # Skip all updates when globally paused (during motion planning)
        if self._updates_paused:
            return
        
        # Wait for camera calibration
        try:
            self.tf_buffer.lookup_transform(
                'base_link', 'camera_color_frame', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.0)
            )
        except Exception:
            return  # Camera not calibrated yet
        
        for detection in msg.detections:
            tag_id = detection.id
            tag_frame = f'{detection.family}:{tag_id}'
            
            # Get object config for this tag
            obj_config = self.objects.get(tag_id, self.objects.get('default'))
            if not obj_config:
                self.get_logger().warn(f'No config for tag {tag_id}')
                continue
            
            try:
                transform = self.tf_buffer.lookup_transform(
                    'base_link', tag_frame, rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.1)
                )
                
                # Build pose
                pose = PoseStamped()
                pose.header.frame_id = 'base_link'
                pose.pose.position.x = transform.transform.translation.x
                pose.pose.position.y = transform.transform.translation.y
                pose.pose.position.z = transform.transform.translation.z
                pose.pose.orientation = transform.transform.rotation
                
                # Apply offsets for bins
                if obj_config['type'] == 'bin':
                    pose.pose.position.x += obj_config.get('offset_x', 0.0)
                    pose.pose.position.y += obj_config.get('offset_y', 0.0)
                    pose.pose.position.z += obj_config.get('offset_z', 0.0)
                
                # Apply Z offset for cubes (tag on top, need base position)
                elif obj_config['type'] == 'cube':
                    pose.pose.position.z -= 0.0175  # Half of 35mm cube
                
                # Store detected object
                obj_name = obj_config['name']
                # Make cube names unique by tag ID
                if obj_config['type'] == 'cube':
                    obj_name = f'cube_{tag_id}'
                
                # Skip paused objects
                if obj_name in self.paused_objects:
                    continue
                
                # Skip permanently removed objects
                if obj_name in self.permanently_removed:
                    continue

                if obj_name in self.detected_objects:
                    old_pose = self.detected_objects[obj_name]
                    dx = abs(pose.pose.position.x - old_pose.pose.position.x)
                    dy = abs(pose.pose.position.y - old_pose.pose.position.y)
                    dz = abs(pose.pose.position.z - old_pose.pose.position.z)
                    
                    if dx < 0.005 and dy < 0.005 and dz < 0.005:
                        continue  # Skip update, object hasn't moved significantly
                
                if obj_name not in self.detected_objects:
                    self.detected_objects[obj_name] = pose
                    self.get_logger().info(f'✓ {obj_name} at ({pose.pose.position.x:.3f}, '
                                          f'{pose.pose.position.y:.3f}, '
                                          f'{pose.pose.position.z:.3f})')
                    
                    # Publish collision object
                    self.publish_collision_object(obj_name, obj_config, pose)
                else:
                    self.detected_objects[obj_name] = pose
                    # Update collision object
                    self.publish_collision_object(obj_name, obj_config, pose)
            
            except Exception as e:
                self.get_logger().warn(f'TF lookup failed for {tag_frame}: {str(e)[:100]}')


def main(args=None):
    rclpy.init(args=args)
    node = SceneManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
