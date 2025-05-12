#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path

class OpenLoopPathFollower(Node):
    def __init__(self):
        super().__init__('open_loop_path_follower')

        # parameters
        self.declare_parameter('linear_speed', 0.3)
        self.declare_parameter('angular_speed', 0.5)
        self.declare_parameter('rate', 20.0)  # Hz

        self.v = self.get_parameter('linear_speed').value
        self.w = self.get_parameter('angular_speed').value
        rate = self.get_parameter('rate').value
        self.timer_period = 1.0 / rate

        # internal state
        self.segments = []  # list of (angle, distance)
        self.idx = 0
        self.phase = 0       # 0 = rotate, 1 = drive
        self.phase_start = None

        # publisher & subscription
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_sub = self.create_subscription(
            Path, '/global_plan', self.path_callback, 10)

        # timer for open-loop execution
        self.create_timer(self.timer_period, self.timer_callback)
        self.get_logger().info('OpenLoopPathFollower started (using Path)')

    def path_callback(self, msg: Path):
        poses = msg.poses
        self.segments.clear()
        if len(poses) < 2:
            self.get_logger().warn('Received path with less than 2 poses. Skipping.')
            return

        for i in range(len(poses) - 1):
            p0 = poses[i].pose.position
            p1 = poses[i + 1].pose.position
            dx = p1.x - p0.x
            dy = p1.y - p0.y
            angle = math.atan2(dy, dx)
            dist = math.hypot(dx, dy)
            self.segments.append((angle, dist))

        self.idx = 0
        self.phase = 0
        self.phase_start = self.get_clock().now().nanoseconds * 1e-9
        self.get_logger().info(f'Loaded {len(self.segments)} segments from path.')

    def timer_callback(self):
        cmd = Twist()

        if self.idx >= len(self.segments):
            self.cmd_pub.publish(cmd)
            return

        target_angle, target_dist = self.segments[self.idx]
        now = self.get_clock().now().nanoseconds * 1e-9
        elapsed = now - (self.phase_start or now)

        if self.phase == 0:
            # rotate in place
            cmd.angular.z = math.copysign(self.w, target_angle)
            if elapsed >= abs(target_angle) / self.w:
                self.phase = 1
                self.phase_start = now
        else:
            # drive straight
            cmd.linear.x = self.v
            if elapsed >= target_dist / self.v:
                self.idx += 1
                self.phase = 0
                self.phase_start = now

        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = OpenLoopPathFollower()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

