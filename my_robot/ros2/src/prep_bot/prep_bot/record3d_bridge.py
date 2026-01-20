#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
from record3d import Record3DStream
import cv2
import time

# uses Record3D app for images, https://record3d.app
# depth may be included later

class Record3DBridge(Node):
    def __init__(self):
        super().__init__('record3d_bridge')
        
        self.bridge = CvBridge()
        self._is_running = True
        self._frame_count = 0
        self._last_log_time = time.time()
        self._latest_rgb = None
        self._latest_timestamp = None
        
        # Publishers with queue size 1 to prevent buffering
        self.color_pub = self.create_publisher(Image, '/camera/color/image_raw', 1)
        self.depth_pub = self.create_publisher(Image, '/camera/depth/image_raw', 1)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/camera/color/camera_info', 1)
        
        # Record3D stream
        self.stream = Record3DStream()
        # Set callback to receive frames (this is required for the event loop!)
        self.stream.on_new_frame = self.on_new_frame
        
        self.get_logger().info('Searching for iPhone via USB...')
        
        # Get list of connected devices
        devices = self.stream.get_connected_devices()
        
        if len(devices) == 0:
            self.get_logger().error('No iPhone found! Make sure:')
            self.get_logger().error('1. iPhone is connected via USB')
            self.get_logger().error('2. Record3D app is running')
            self.get_logger().error('3. USB streaming is enabled in app')
            return
        
        self.get_logger().info(f'Found {len(devices)} device(s)')
        
        # Connect to first device
        device = devices[0]
        self.get_logger().info(f'Connecting to device: {device.udid} (Product ID: {device.product_id})')
        
        if self.stream.connect(device):
            self.get_logger().info('Connected successfully! Streaming...')
            # Create a timer to publish frames at 30 Hz from the latest received frame
            self.timer = self.create_timer(1.0 / 30.0, self.publish_frame)
        else:
            self.get_logger().error('Failed to connect to device')

    def on_new_frame(self):
        """Callback when Record3D receives a new frame"""
        try:
            rgb = self.stream.get_rgb_frame()
            if rgb is not None and rgb.size > 0 and len(rgb.shape) == 3:
                self._latest_rgb = rgb
                self._latest_timestamp = self.get_clock().now().to_msg()
        except Exception as e:
            pass

    def publish_frame(self):
        """Publish the latest received frame"""
        if not self._is_running:
            return
        
        try:
            # Log FPS every 5 seconds
            self._frame_count += 1
            current_time = time.time()
            
            # Check if we have a frame to publish
            if self._latest_rgb is None:
                return
            
            # Use the stored frame and timestamp
            rgb = self._latest_rgb
            timestamp = self._latest_timestamp if self._latest_timestamp else self.get_clock().now().to_msg()
            
            # Count successful publishes
            self._publish_count = getattr(self, '_publish_count', 0) + 1
            
            # Log stats every 1 second at DEBUG level
            if current_time - self._last_log_time > 1.0:
                fps = self._frame_count / (current_time - self._last_log_time)
                actual_fps = self._publish_count / (current_time - self._last_log_time)
                self.get_logger().debug(f'Publishing at {actual_fps:.1f} fps (timer: {fps:.1f} Hz)')
                self._frame_count = 0
                self._publish_count = 0
                self._last_log_time = current_time
            
            # Downsample for faster processing (optional but helps)
            # rgb = cv2.resize(rgb, (rgb.shape[1]//2, rgb.shape[0]//2))
            
            # Convert RGB to BGR for ROS
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            color_msg = self.bridge.cv2_to_imgmsg(bgr, encoding='bgr8')
            color_msg.header.stamp = timestamp
            color_msg.header.frame_id = 'camera_color_frame'
            self.color_pub.publish(color_msg)
            
            # Skip depth publishing to reduce bandwidth (comment out if needed)
            # depth = self.stream.get_depth_frame()
            # if depth is not None:
            #     depth_mm = (depth * 1000).astype(np.uint16)
            #     depth_msg = self.bridge.cv2_to_imgmsg(depth_mm, encoding='16UC1')
            #     depth_msg.header.stamp = timestamp
            #     depth_msg.header.frame_id = 'camera_depth_frame'
            #     self.depth_pub.publish(depth_msg)
            
            # Publish camera info
            intrinsics = self.stream.get_intrinsic_mat()
            if intrinsics is not None:
                camera_info = CameraInfo()
                camera_info.header.stamp = timestamp
                camera_info.header.frame_id = 'camera_color_frame'
                camera_info.width = rgb.shape[1]
                camera_info.height = rgb.shape[0]
                
                # K is a 3x3 matrix in row-major order: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
                camera_info.k = [
                    intrinsics.fx, 0.0, intrinsics.tx,
                    0.0, intrinsics.fy, intrinsics.ty,
                    0.0, 0.0, 1.0
                ]
                
                # P is 3x4 projection matrix
                camera_info.p = [
                    intrinsics.fx, 0.0, intrinsics.tx, 0.0,
                    0.0, intrinsics.fy, intrinsics.ty, 0.0,
                    0.0, 0.0, 1.0, 0.0
                ]
                
                # Distortion model - assume plumb_bob (standard pinhole)
                # this is likely not accurate
                camera_info.distortion_model = 'plumb_bob'
                camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]  # No distortion (iPhone pre-corrects)
                
                # Rectification matrix (identity for monocular)
                camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
                
                self.camera_info_pub.publish(camera_info)
        except Exception as e:
            if self._is_running:
                self.get_logger().error(f'Error polling frame: {e}', throttle_duration_sec=5.0)
    
    def __del__(self):
        """Cleanup when node is destroyed"""
        self._is_running = False
        if hasattr(self, 'stream'):
            try:
                self.stream.disconnect()
            except:
                pass

def main(args=None):
    rclpy.init(args=args)
    node = Record3DBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        node._is_running = False
        if hasattr(node, 'stream'):
            try:
                node.stream.disconnect()
            except:
                pass
        try:
            node.destroy_node()
        except:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()