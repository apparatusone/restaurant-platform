#!/usr/bin/env python3
"""
Pick and Place Controller

Detects objects and bins via AprilTag, executes pick-and-place operations.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from apriltag_msgs.msg import AprilTagDetectionArray
from tf2_ros import TransformListener, Buffer
from ament_index_python.packages import get_package_share_directory
import rclpy.time
import rclpy.duration
import yaml
import os


class PickPlaceNode(Node):
    def __init__(self):
        super().__init__('pick_place')
        
        # Load object config
        pkg_share = get_package_share_directory('prep_bot')
        config_path = os.path.join(pkg_share, 'config', 'tags.yaml')
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
            self.objects = config.get('objects', {})
        
        self.get_logger().info(f'Loaded {len(self.objects)} object definitions')
        
        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscribers
        self.create_subscription(AprilTagDetectionArray, '/detections', self.tag_callback, 10)
        
        # Detected objects (keyed by name)
        self.detected_objects = {}
        
        self.get_logger().info('Pick and Place Controller Ready')
        self.get_logger().info('Waiting for AprilTag detections...')
    
    def tag_callback(self, msg):
        if not msg.detections:
            return
        
        for detection in msg.detections:
            tag_id = detection.id
            tag_frame = f'{detection.family}:{tag_id}'
            
            # Get object config for this tag
            obj_config = self.objects.get(tag_id, self.objects.get('default'))
            if not obj_config:
                continue
            
            try:
                transform = self.tf_buffer.lookup_transform(
                    'base_link', tag_frame, rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5)
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
                    pose.pose.position.z -= 0.015  # Half cube height
                    pose.pose.orientation.w = 1.0  # Reset orientation
                
                # Store detected object
                obj_name = obj_config['name']
                if obj_name not in self.detected_objects:
                    self.detected_objects[obj_name] = pose
                    self.get_logger().info(f'✓ {obj_name} at ({pose.pose.position.x:.3f}, '
                                          f'{pose.pose.position.y:.3f}, '
                                          f'{pose.pose.position.z:.3f})')
                else:
                    self.detected_objects[obj_name] = pose
            
            except Exception:
                pass  # TF lookup failed


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
