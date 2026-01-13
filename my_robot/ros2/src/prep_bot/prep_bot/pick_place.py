#!/usr/bin/env python3
"""
Pick and Place Controller

Detects objects and bins via AprilTag, executes pick-and-place operations.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from apriltag_msgs.msg import AprilTagDetectionArray
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive, Mesh, MeshTriangle
from geometry_msgs.msg import Point
from tf2_ros import TransformListener, Buffer
from ament_index_python.packages import get_package_share_directory
from scipy.spatial.transform import Rotation
import rclpy.time
import rclpy.duration
import yaml
import os
import struct


def load_stl_mesh(filepath):
    """Load an STL file and return a Mesh message."""
    mesh = Mesh()
    
    with open(filepath, 'rb') as f:
        f.read(80)  # Skip header
        num_triangles = struct.unpack('<I', f.read(4))[0]
        
        vertices = []
        vertex_map = {}
        
        for _ in range(num_triangles):
            f.read(12)  # Skip normal
            
            triangle = MeshTriangle()
            indices = []
            
            for _ in range(3):
                vx, vy, vz = struct.unpack('<fff', f.read(12))
                vertex_key = (vx, vy, vz)
                
                if vertex_key not in vertex_map:
                    p = Point()
                    p.x = float(vx)
                    p.y = float(vy)
                    p.z = float(vz)
                    vertex_map[vertex_key] = len(vertices)
                    vertices.append(p)
                
                indices.append(vertex_map[vertex_key])
            
            triangle.vertex_indices = indices
            mesh.triangles.append(triangle)
            f.read(2)  # Skip attribute
    
    mesh.vertices = vertices
    return mesh


class PickPlaceNode(Node):
    def __init__(self):
        super().__init__('pick_place')
        
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
            self.get_logger().info(f'Loaded bin mesh: {len(self.bin_mesh.triangles)} triangles')
        else:
            self.bin_mesh = None
            self.get_logger().warn(f'Bin mesh not found: {mesh_path}')
        
        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Publishers
        self.collision_pub = self.create_publisher(CollisionObject, '/collision_object', 10)
        
        # Subscribers
        self.create_subscription(AprilTagDetectionArray, '/detections', self.tag_callback, 10)
        
        # Detected objects (keyed by name)
        self.detected_objects = {}
        
        self.get_logger().info('Pick and Place Controller Ready')
        self.get_logger().info('Waiting for AprilTag detections...')
    
    def publish_collision_object(self, obj_name, obj_config, pose):
        """Publish collision object to MoveIt planning scene"""
        # Skip objects without collision shapes
        if 'collision_shape' not in obj_config:
            return
        
        obj = CollisionObject()
        obj.header.stamp = self.get_clock().now().to_msg()
        obj.header.frame_id = 'base_link'
        obj.id = obj_name
        
        if obj_config.get('collision_shape') == 'mesh' and self.bin_mesh:
            # Flatten X/Y rotation, keep Z rotation
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
            box.dimensions = obj_config.get('dimensions', [0.035, 0.035, 0.035])
            obj.primitives = [box]
            obj.primitive_poses = [pose.pose]
        
        obj.operation = CollisionObject.ADD
        self.collision_pub.publish(obj)
    
    def tag_callback(self, msg):
        if not msg.detections:
            return
        
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
                    pose.pose.orientation.w = 1.0  # Reset orientation
                
                # Store detected object
                obj_name = obj_config['name']
                # Make cube names unique by tag ID
                if obj_config['type'] == 'cube':
                    obj_name = f'cube_{tag_id}'
                
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
    node = PickPlaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
