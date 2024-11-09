import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry  # For getting robot's current position
from std_msgs.msg import Bool

class MoveToGoal(Node):
    def __init__(self):
        super().__init__('move_to_goal')
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_reached_pub = self.create_publisher(Bool, '/goal_status', 10)

        self.goal = None
        self.current_position = None
        self.current_orientation = None

    def goal_callback(self, msg):
        self.goal = msg.pose
        self.get_logger().info(f"Received goal: x={self.goal.position.x}, y={self.goal.position.y}")

    def odom_callback(self, msg):
        # Update current position and orientation from odometry
        self.current_position = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        self.current_orientation = self.quaternion_to_euler(orientation_q)

        # Move toward the goal if it exists
        if self.goal:
            self.move_toward_goal()

    def quaternion_to_euler(self, q):
        # Convert quaternion (q) to euler angles (roll, pitch, yaw)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def move_toward_goal(self):
        goal_x = self.goal.position.x
        goal_y = self.goal.position.y

        # Calculate distance and angle to the goal
        dx = goal_x - self.current_position.x
        dy = goal_y - self.current_position.y
        distance = math.sqrt(dx**2 + dy**2)
        target_angle = math.atan2(dy, dx)

        # Define the tolerance for reaching the goal
        distance_tolerance = 0.1  # meters
        angle_tolerance = 0.1  # radians

        # Calculate the angle difference between the robot's orientation and the target
        angle_diff = target_angle - self.current_orientation
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi  # Normalize to [-pi, pi]

        twist_msg = Twist()

        # Rotate if angle difference is greater than tolerance
        if abs(angle_diff) > angle_tolerance:
            twist_msg.angular.z = 0.7 * angle_diff  # Adjust rotation speed
            self.get_logger().info("Rotating to face the goal")
        # Move forward if aligned and within distance tolerance
        elif distance > distance_tolerance:
            twist_msg.linear.x = 0.3  # Adjust forward speed
            self.get_logger().info("Moving toward the goal")
        else:
            # Stop and publish goal reached status
            self.goal_reached_pub.publish(Bool(data=True))
            self.goal = None  # Reset goal to prevent further movement
            self.get_logger().info("Goal reached")
            return

        # Publish the movement command
        self.cmd_vel_pub.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    move_to_goal = MoveToGoal()
    rclpy.spin(move_to_goal)
    rclpy.shutdown()
