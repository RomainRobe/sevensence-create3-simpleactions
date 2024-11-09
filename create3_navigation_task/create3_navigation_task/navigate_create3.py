# navigate_create3.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from irobot_create_msgs.action import Undock, Dock
from irobot_create_msgs.msg import DockStatus
from rclpy.action import ActionClient
import yaml
from ament_index_python.packages import get_package_share_directory
import os
from std_msgs.msg import Bool
import math
import time
from enum import Enum

class NavigationState(Enum):
    NAVIGATE_TO_GOAL = 1
    SCAN_FOR_DOCK = 2
    FIBONACCI_SPIRAL = 3

class Create3Navigator(Node):
    def __init__(self):
        super().__init__('navigate_create3')
        self.undock_client = ActionClient(self, Undock, '/undock')
        self.dock_client = ActionClient(self, Dock, '/dock')
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.goal_status_sub = self.create_subscription(Bool, '/goal_status', self.goal_status_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.dock_status_sub = self.create_subscription(DockStatus, '/dock_status', self.dock_status_callback, 10)

        self.target_pose = self.load_target_pose()['target_pose']
        self.current_goal_index = 0
        self.state = NavigationState.NAVIGATE_TO_GOAL  # Set initial state
        self.waypoints = self.generate_s_shape_waypoints({"x": 0.0, "y": 0.0}, self.target_pose)
        self.goal_reached = True
        self.dock_available = False  # Docking station status
        self.fibonacci_index = 1
        self.undock()

    def load_target_pose(self):
        config_path = os.path.join(
            get_package_share_directory('create3_navigation_task'),
            'config',
            'target_pose.yaml'
        )
        with open(config_path, 'r') as file:
            data = yaml.safe_load(file)
        return data

    def undock(self):
        undock_goal = Undock.Goal()
        self.undock_client.wait_for_server()
        self.get_logger().info("Sending undock goal...")
        self.undock_client.send_goal_async(undock_goal).add_done_callback(self.on_undock_goal_response)

    def on_undock_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Undock goal rejected!")
            return
        goal_handle.get_result_async().add_done_callback(self.on_undock_complete)

    def on_undock_complete(self, future):
        if future.result().status == 4:  # SUCCEEDED
            self.get_logger().info("Undocking complete, navigating to target position")
            self.send_next_goal()
        else:
            self.get_logger().error("Undocking failed")

    def dock_status_callback(self, msg):
        self.dock_available = msg.dock_visible
        if self.dock_available:
            self.get_logger().info("Docking station detected")
            # self.dock()

    def goal_status_callback(self, msg):
        self.goal_reached = msg.data
        if self.goal_reached:
            if self.state == NavigationState.NAVIGATE_TO_GOAL:
                self.send_next_goal()
            elif self.state == NavigationState.SCAN_FOR_DOCK:
                self.perform_360_scan()
            elif self.state == NavigationState.FIBONACCI_SPIRAL:
                self.fibonacci_spiral_exploration()
    
    def generate_s_shape_waypoints(self, start, target):
        waypoints = []
        num_waypoints = 10
        x_step = (target['x'] - start['x']) / num_waypoints
        y_amplitude = abs(target['y'] - start['y']) / 4

        for i in range(1, num_waypoints + 1):
           x = start['x'] + i * x_step
           y = start['y'] + y_amplitude * math.sin(i * math.pi / (num_waypoints // 2))
           waypoints.append({"x": x, "y": y})

        waypoints.append(target)
        return waypoints

    def send_next_goal(self):
        if self.current_goal_index >= len(self.waypoints):
            self.get_logger().info("All waypoints reached, initiating 360 scan for docking station.")
            self.state = NavigationState.SCAN_FOR_DOCK
            self.perform_360_scan()
            return

        point = self.waypoints[self.current_goal_index]
        target_msg = PoseStamped()
        target_msg.header.frame_id = "map"
        target_msg.pose.position.x = point['x']
        target_msg.pose.position.y = point['y']
        self.goal_pub.publish(target_msg)
        self.get_logger().info(f"Published goal to waypoint: x={point['x']}, y={point['y']}")
        self.current_goal_index += 1

    def perform_360_scan(self):
        self.get_logger().info("Starting 360-degree scan for docking station")
        self.state = NavigationState.SCAN_FOR_DOCK
        self.rotation_steps_completed = 0  # Reset step counter

        # Set up a timer to rotate incrementally
        self.scan_rotation_timer = self.create_timer(0.23, self.incremental_rotation_step)

    def incremental_rotation_step(self):
        if self.dock_available:
            # Stop rotation and dock if docking station is available
            self.get_logger().info("Docking station detected during scan, docking now!")
            self.scan_rotation_timer.cancel()
            self.dock()
            return

        if self.rotation_steps_completed >= 72:
            # Completed full 360-degree rotation (assuming 72 steps of 5 degrees each)
            self.scan_rotation_timer.cancel()
            self.get_logger().info("Docking station not found after full scan, switching to Fibonacci spiral")
            self.state = NavigationState.FIBONACCI_SPIRAL
            self.fibonacci_spiral_exploration()
            return

        # Publish a rotation command for a small incremental rotation
        twist = Twist()
        twist.angular.z = 0.5  # Adjust rotation speed if needed
        self.cmd_vel_pub.publish(twist)
        self.rotation_steps_completed += 1

    def fibonacci_spiral_exploration(self):
        if self.dock_available:
            self.get_logger().info("Docking station detected during scan, docking now!")
            self.dock()
            return

        self.get_logger().info("Starting Fibonacci spiral exploration.")
        fib_value = self.fibonacci_index
        self.fibonacci_index += 1

        angle = math.radians(30 * fib_value)
        distance = fib_value * 0.05

        target_x = self.target_pose['x'] + distance * math.cos(angle)
        target_y = self.target_pose['y'] + distance * math.sin(angle)

        target_msg = PoseStamped()
        target_msg.header.frame_id = "map"
        target_msg.pose.position.x = target_x
        target_msg.pose.position.y = target_y
        self.goal_pub.publish(target_msg)
        self.get_logger().info(f"Exploring at spiral position: x={target_x}, y={target_y}")

    def dock(self):
        self.get_logger().info("Initiating docking procedure")
        dock_goal = Dock.Goal()
        self.dock_client.wait_for_server()
        self.dock_client.send_goal(dock_goal)

def main(args=None):
    rclpy.init(args=args)
    navigator = Create3Navigator()
    rclpy.spin(navigator)
    rclpy.shutdown()
