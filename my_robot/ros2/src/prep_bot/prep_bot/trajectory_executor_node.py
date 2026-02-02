#!/usr/bin/env python3
"""
Trajectory Executor Node - Sends all waypoints then starts execution

Receives trajectories from MoveIt and sends WAYPOINT commands to the microcontroller.
Buffer size 500 - sends all waypoints before starting.

Topics:
  Subscribes: /motor_feedback (String) - Responses from microcontroller
  Publishes:  /motor_command (String)  - Commands to microcontroller
  
Actions:
  Server: robot_arm_controller/follow_joint_trajectory - Execute planned trajectory
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String
from std_srvs.srv import SetBool
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory
import threading
import time


class TrajectoryExecutorNode(Node):
    def __init__(self):
        super().__init__('trajectory_executor')
        
        self.callback_group = ReentrantCallbackGroup()
        
        # Declare parameters (no defaults - must be from config)
        self.declare_parameter('stm32_buffer_size')
        self.declare_parameter('command_timeout')
        self.declare_parameter('trajectory_timeout_margin')
        self.declare_parameter('waypoint_log_interval')
        self.declare_parameter('joint_map.linear_joint')
        self.declare_parameter('joint_map.shoulder_joint')
        self.declare_parameter('joint_map.elbow_joint')
        
        # Validate
        required = [
            'stm32_buffer_size', 'command_timeout', 'trajectory_timeout_margin',
            'waypoint_log_interval', 'joint_map.linear_joint',
            'joint_map.shoulder_joint', 'joint_map.elbow_joint'
        ]
        missing = [p for p in required if not self.has_parameter(p)]
        if missing:
            self.get_logger().error(f'Required parameters missing: {", ".join(missing)}')
            raise RuntimeError('Missing required configuration')
        
        # Get parameters
        self.expected_buffer_size = self.get_parameter('stm32_buffer_size').value
        self.command_timeout = self.get_parameter('command_timeout').value
        self.trajectory_timeout_margin = self.get_parameter('trajectory_timeout_margin').value
        self.waypoint_log_interval = self.get_parameter('waypoint_log_interval').value
        
        self.cmd_pub = self.create_publisher(String, 'motor_command', 10)
        
        self.feedback_sub = self.create_subscription(
            String,
            'motor_feedback',
            self.feedback_callback,
            10,
            callback_group=self.callback_group
        )
        
        # Service client to pause/resume hardware_interface STATUS polling
        self.pause_status_client = self.create_client(
            SetBool,
            '/hardware_interface/pause_status',
            callback_group=self.callback_group
        )
        
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            'robot_arm_controller/follow_joint_trajectory',
            self.execute_trajectory_callback,
            callback_group=self.callback_group
        )
        
        self.last_response = None
        self.response_event = threading.Event()
        self.buffer_size = None
        self._executing = False  # Track if trajectory is executing
        
        # Joint name mapping - rotation_joint is FIXED (motor broken)
        self.joint_map = {
            'linear_joint': self.get_parameter('joint_map.linear_joint').value,
            'shoulder_joint': self.get_parameter('joint_map.shoulder_joint').value,
            'elbow_joint': self.get_parameter('joint_map.elbow_joint').value,
        }
        
        self.get_logger().info('Trajectory Executor ready')

    def feedback_callback(self, msg):
        response = msg.data.strip()
        self.last_response = response
        
        if response.startswith('ACK'):
            parts = response.split()
            if len(parts) > 1:
                try:
                    self.buffer_size = int(parts[1])
                except ValueError:
                    pass
            # Only set event for ACK if we're waiting for one (not during execution)
            if not hasattr(self, '_executing') or not self._executing:
                self.response_event.set()
        elif response == 'DONE' or response == 'STOPPED' or response.startswith('ERROR'):
            self.response_event.set()

    def send_command(self, cmd: str):
        msg = String()
        msg.data = cmd
        self.cmd_pub.publish(msg)

    def send_waypoint(self, positions: list, velocities: list, time_ms: int):
        cmd = f'WAYPOINT {positions[0]:.6f} {positions[1]:.6f} {positions[2]:.6f} {positions[3]:.6f} {velocities[0]:.6f} {velocities[1]:.6f} {velocities[2]:.6f} {velocities[3]:.6f} {time_ms}'
        msg = String()
        msg.data = cmd
        self.cmd_pub.publish(msg)

    def trajectory_to_joint_positions(self, trajectory: JointTrajectory, point_idx: int) -> list:
        positions = [0.0, 0.0, 0.0, 0.0]
        point = trajectory.points[point_idx]
        
        for i, name in enumerate(trajectory.joint_names):
            if name in self.joint_map:
                idx = self.joint_map[name]
                positions[idx] = point.positions[i]
        
        return positions

    def trajectory_to_joint_velocities(self, trajectory: JointTrajectory, point_idx: int) -> list:
        velocities = [0.0, 0.0, 0.0, 0.0]
        point = trajectory.points[point_idx]
        
        if len(point.velocities) > 0:
            for i, name in enumerate(trajectory.joint_names):
                if name in self.joint_map:
                    idx = self.joint_map[name]
                    velocities[idx] = point.velocities[i]
        
        return velocities
    
    def _resume_status_polling(self):
        """Resume STATUS polling in hardware_interface"""
        if self.pause_status_client.wait_for_service(timeout_sec=0.5):
            req = SetBool.Request()
            req.data = False
            future = self.pause_status_client.call_async(req)
            # Don't wait for response
        else:
            self.get_logger().warn('pause_status service not available for resume')

    async def execute_trajectory_callback(self, goal_handle):
        self.get_logger().info('Received trajectory')
        
        trajectory = goal_handle.request.trajectory
        num_points = len(trajectory.points)
        
        if num_points == 0:
            self.get_logger().error('Empty trajectory')
            goal_handle.abort()
            return FollowJointTrajectory.Result()
        
        self.get_logger().info(f'{num_points} waypoints')
        
        # Pause STATUS polling during trajectory execution
        if self.pause_status_client.wait_for_service(timeout_sec=1.0):
            req = SetBool.Request()
            req.data = True
            future = self.pause_status_client.call_async(req)
        else:
            self.get_logger().warn('pause_status service not available')
        
        # Clear buffer - get buffer size from STM32
        self.response_event.clear()
        self.buffer_size = None
        self.send_command('CLEAR_TRAJ')
        if not self.response_event.wait(self.command_timeout):
            self.get_logger().error('No ACK for CLEAR_TRAJ')
            self._resume_status_polling()
            goal_handle.abort()
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            return result
        
        if self.buffer_size is None:
            self.get_logger().error('STM32 did not report buffer size')
            self._resume_status_polling()
            goal_handle.abort()
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            return result
        
        if num_points > self.buffer_size:
            self.get_logger().error(f'Trajectory too large: {num_points} > {self.buffer_size}')
            self._resume_status_polling()
            goal_handle.abort()
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            return result
        
        # Send all waypoints
        for i in range(num_points):
            if goal_handle.is_cancel_requested:
                self.send_command('STOP')
                self._resume_status_polling()
                goal_handle.canceled()
                return FollowJointTrajectory.Result()
            
            positions = self.trajectory_to_joint_positions(trajectory, i)
            velocities = self.trajectory_to_joint_velocities(trajectory, i)
            point = trajectory.points[i]
            time_ms = int(point.time_from_start.sec * 1000 + point.time_from_start.nanosec / 1e6)
            
            self.send_waypoint(positions, velocities, time_ms)
            
            if i % self.waypoint_log_interval == 0:
                self.get_logger().info(f'Sent {i}/{num_points}')
        
        self.get_logger().info(f'All {num_points} waypoints sent')
        
        # Start execution
        self.response_event.clear()
        self.send_command('START_TRAJ')
        if not self.response_event.wait(self.command_timeout):
            self.get_logger().error('No ACK for START_TRAJ')
            self.send_command('STOP')
            self._resume_status_polling()
            goal_handle.abort()
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            return result
        
        self.get_logger().info('Execution started')
        self._executing = True
        
        # Wait for DONE
        total_time_ms = int(trajectory.points[-1].time_from_start.sec * 1000 + 
                           trajectory.points[-1].time_from_start.nanosec / 1e6)
        timeout = (total_time_ms / 1000.0) + self.trajectory_timeout_margin
        
        self.get_logger().info(f'Waiting for DONE (timeout: {timeout:.1f}s, trajectory: {total_time_ms/1000.0:.1f}s)')
        
        self.response_event.clear()
        self.last_response = None
        if self.response_event.wait(timeout):
            if self.last_response == 'DONE':
                self.get_logger().info('Trajectory complete')
                self._executing = False
                self._resume_status_polling()
                time.sleep(0.25)
                goal_handle.succeed()
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                return result
        
        self.get_logger().error('Timeout or error')
        self._executing = False
        self._resume_status_polling()
        goal_handle.abort()
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
        return result


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryExecutorNode()
    
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
