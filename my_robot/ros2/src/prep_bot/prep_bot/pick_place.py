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
import rclpy.time
import rclpy.duration


class PickPlaceNode(Node):
    def __init__(self):
        super().__init__('pick_place')
        
        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscribers
        self.create_subscription(AprilTagDetectionArray, '/detections', self.tag_callback, 10)
        
        self.get_logger().info('Pick and Place Controller Ready')
        self.get_logger().info('Waiting for AprilTag detections...')
    
    def tag_callback(self, msg):
        if not msg.detections:
            return
        
        for detection in msg.detections:
            tag_id = detection.id
            tag_frame = f'{detection.family}:{tag_id}'
            
            try:
                transform = self.tf_buffer.lookup_transform(
                    'base_link', tag_frame, rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5)
                )
            
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
