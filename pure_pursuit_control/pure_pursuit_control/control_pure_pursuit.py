#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
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
        # list of (target_angle, travel_distance) segments
        self.segments = []
        self.idx = 0         # current segment index
        self.phase = 0       # 0 = rotating, 1 = driving
        self.phase_start = None

        # publisher & subscription
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_sub = self.create_subscription(
            Path, '/global_plan', self.path_callback, 10)

        # timer for open-loop execution
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.get_logger().info('OpenLoopPathFollower started')

    def path_callback(self, msg: Path):
        # Build segments from consecutive path points
        self.segments.clear()
        poses = msg.poses
        for i in range(len(poses) - 1):
            x0 = poses[i].pose.position.x
            y0 = poses[i].pose.position.y
            x1 = poses[i+1].pose.position.x
            y1 = poses[i+1].pose.position.y
            dx = x1 - x0
            dy = y1 - y0
            angle = math.atan2(dy, dx)
            dist = math.hypot(dx, dy)
            self.segments.append((angle, dist))

        self.idx = 0
        self.phase = 0
        self.phase_start = self.get_clock().now().nanoseconds * 1e-9
        self.get_logger().info(
            f"Received path → {len(self.segments)} segments")

    def timer_callback(self):
        cmd = Twist()

        # no segments or done
        if self.idx >= len(self.segments):
            # stop robot
            self.cmd_pub.publish(cmd)
            return

        target_angle, target_dist = self.segments[self.idx]
        now = self.get_clock().now().nanoseconds * 1e-9
        elapsed = now - self.phase_start

        if self.phase == 0:
            # ROTATE phase
            cmd.angular.z = math.copysign(self.w, target_angle)
            # time needed = |angle| / w
            if elapsed >= abs(target_angle) / self.w:
                # switch to DRIVE phase
                self.phase = 1
                self.phase_start = now

        else:
            # DRIVE phase
            cmd.linear.x = self.v
            # time needed = distance / v
            if elapsed >= target_dist / self.v:
                # move to next segment
                self.idx += 1
                self.phase = 0
                self.phase_start = now

        # publish cmd_vel every timer tick
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = OpenLoopPathFollower()
    try:
        rclpy.spin(node

