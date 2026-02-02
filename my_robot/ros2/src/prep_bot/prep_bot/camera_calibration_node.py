#!/usr/bin/env python3
"""
Camera Calibration Node

Calibrates camera position by detecting AprilTag on the linear carriage.
Publishes static TF: world -> camera_color_frame
"""
import rclpy
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import TransformStamped
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster, TransformListener, Buffer
from scipy.spatial.transform import Rotation
import numpy as np
import rclpy.time
import rclpy.duration


class CameraCalibrationNode(Node):
    def __init__(self):
        super().__init__('camera_calibration')
        
        self.declare_parameter('calibration_tag', 12)
        self.declare_parameter('calibration_samples', 10)
        self.declare_parameter('tag_c_offset_x', 0.0)
        self.declare_parameter('tag_c_offset_y', 0.0)
        self.declare_parameter('tag_c_offset_z', 0.0)
        self.declare_parameter('carriage_home_x', 0.0)
        self.declare_parameter('carriage_home_y', 0.0)
        self.declare_parameter('carriage_home_z', 0.0)
        
        # Get parameter values
        self.num_samples = self.get_parameter('calibration_samples').value
        self.tag_c_offset = np.array([
            self.get_parameter('tag_c_offset_x').value,
            self.get_parameter('tag_c_offset_y').value,
            self.get_parameter('tag_c_offset_z').value
        ])
        self.carriage_home = np.array([
            self.get_parameter('carriage_home_x').value,
            self.get_parameter('carriage_home_y').value,
            self.get_parameter('carriage_home_z').value
        ])
        
        # State
        self.calibration_samples = []
        self.calibrated = False
        
        # Calibration status service
        self.create_service(Trigger, '/camera_calibration_status', self.handle_status_request)
        
        # TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # TF broadcaster
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        
        # Subscribe to tag detections
        self.detection_sub = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.detection_callback,
            10
        )
        
        self.get_logger().info('Camera calibration node started')
        self.get_logger().debug(f'Collecting {self.num_samples} samples of calibration tag...')
    
    def detection_callback(self, msg):
        if self.calibrated:
            return
        
        for detection in msg.detections:
            if detection.id == self.get_parameter('calibration_tag').value:
                tag_frame = f'{detection.family}:{detection.id}'
                
                try:
                    # Look up transform from camera to tag (apriltag_node publishes this)
                    transform = self.tf_buffer.lookup_transform(
                        'camera_color_frame', tag_frame, rclpy.time.Time(),
                        timeout=rclpy.duration.Duration(seconds=0.5)
                    )
                    
                    # Store pose (tag in camera frame)
                    self.calibration_samples.append({
                        'position': np.array([
                            transform.transform.translation.x,
                            transform.transform.translation.y,
                            transform.transform.translation.z
                        ]),
                        'orientation': np.array([
                            transform.transform.rotation.x,
                            transform.transform.rotation.y,
                            transform.transform.rotation.z,
                            transform.transform.rotation.w
                        ])
                    })
                    
                    self.get_logger().debug(f'Calibration sample {len(self.calibration_samples)}/{self.num_samples}')
                    
                    # Check if we have enough samples
                    if len(self.calibration_samples) >= self.num_samples:
                        self.compute_calibration()
                        self.calibrated = True
                    
                except Exception as e:
                    self.get_logger().debug(f'TF lookup failed for {tag_frame}: {e}')
                
                break
    
    def compute_calibration(self):
        """Compute camera transform from tag detections."""
        # Average position and orientation
        positions = np.array([s['position'] for s in self.calibration_samples])
        orientations = np.array([s['orientation'] for s in self.calibration_samples])
        
        avg_pos = np.mean(positions, axis=0)
        avg_quat = np.mean(orientations, axis=0)
        avg_quat = avg_quat / np.linalg.norm(avg_quat)  # Normalize quaternion
        
        # Tag position in base_link when carriage is homed
        tag_in_base = self.carriage_home + self.tag_c_offset
        
        # R_tag_in_optical: how tag frame appears in optical frame
        R_tag_in_optical = Rotation.from_quat(avg_quat)
        
        # Tag orientation in world: tag is flat, Z up
        R_tag_in_world = Rotation.identity()
        
        # R_optical_in_world = R_tag_in_world * R_tag_in_optical.inv()
        R_optical_in_world = R_tag_in_world * R_tag_in_optical.inv()
        
        # Position: tag_in_base = R_camera_in_base * tag_in_camera + t_camera_in_base
        # So: t_camera_in_base = tag_in_base - R_camera_in_base * tag_in_camera
        t_camera_in_base = tag_in_base - R_optical_in_world.apply(avg_pos)
        
        # Recreate broadcaster to ensure fresh publish
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        
        # Publish base_link -> camera_color_frame (static, once)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'camera_color_frame'
        t.transform.translation.x = float(t_camera_in_base[0])
        t.transform.translation.y = float(t_camera_in_base[1])
        t.transform.translation.z = float(t_camera_in_base[2])
        
        quat = R_optical_in_world.as_quat()
        t.transform.rotation.x = float(quat[0])
        t.transform.rotation.y = float(quat[1])
        t.transform.rotation.z = float(quat[2])
        t.transform.rotation.w = float(quat[3])
        
        self.tf_broadcaster.sendTransform(t)
        
        self.get_logger().info('=== Camera Calibrated ===')

        # Unsubscribe from detections
        self.destroy_subscription(self.detection_sub)
        self.get_logger().debug(f'Position: ({t_camera_in_base[0]:.4f}, {t_camera_in_base[1]:.4f}, {t_camera_in_base[2]:.4f})')
        self.get_logger().debug(f'Quaternion: ({quat[0]:.4f}, {quat[1]:.4f}, {quat[2]:.4f}, {quat[3]:.4f})')
        self.get_logger().info(f'\n')
    
    def handle_status_request(self, request, response):
        """Handle calibration status service request."""
        response.success = self.calibrated
        response.message = 'Camera calibrated' if self.calibrated else 'Camera not yet calibrated'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = CameraCalibrationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
