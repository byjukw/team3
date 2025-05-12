#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from tf2_ros import TransformListener, Buffer
import tf_transformations


class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')
        self.declare_parameter('lookahead_distance', 0.6)
        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('max_angular_speed', 0.3)
        self.declare_parameter('goal_tolerance', 0.2)
        self.lookahead_distance = self.get_parameter('lookahead_distance').get_parameter_value().double_value
        self.linear_speed = self.get_parameter('linear_speed').get_parameter_value().double_value
        self.max_angular_speed = self.get_parameter('max_angular_speed').get_parameter_value().double_value
        self.goal_tolerance = self.get_parameter('goal_tolerance').get_parameter_value().double_value
        self.path_sub = self.create_subscription(Path,'/global_plan',self.path_callback,10)
        self.cmd_pub = self.create_publisher(Twist,'/cmd_vel',10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.current_path = []
        self.timer = self.create_timer(0.1, self.control_loop)

    def path_callback(self, msg):
        self.current_path = []
        for pose_stamped in msg.poses:
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y
            self.current_path.append((x, y))

    def control_loop(self):
        if len(self.current_path) < 2:
            self.publish_cmd_vel(0.0, 0.0)
            return
        try:
            transform = self.tf_buffer.lookup_transform('map','base_link',rclpy.time.Time(),timeout=rclpy.duration.Duration(seconds=0.2))
        except:
            self.publish_cmd_vel(0.0, 0.0)
            return
        rx = transform.transform.translation.x
        ry = transform.transform.translation.y
        q = transform.transform.rotation
        yaw = self.get_yaw_from_quaternion(q)
        gx, gy = self.current_path[-1]
        d = math.hypot(gx - rx, gy - ry)
        if d < self.goal_tolerance:
            self.publish_cmd_vel(0.0, 0.0)
            return
        lh = None
        for (wx, wy) in self.current_path:
            dx = wx - rx
            dy = wy - ry
            dist = math.hypot(dx, dy)
            if dist >= self.lookahead_distance:
                lh = (wx, wy)
                break
        if lh is None:
            lh = self.current_path[-1]
        err = math.atan2(lh[1] - ry, lh[0] - rx) - yaw
        err = math.atan2(math.sin(err), math.cos(err))
        c = 2.0 * math.sin(err) / self.lookahead_distance
        w = self.linear_speed * c
        if abs(w) > self.max_angular_speed:
            w = self.max_angular_speed * math.copysign(1.0, w)
        self.publish_cmd_vel(self.linear_speed, w)

    def publish_cmd_vel(self, lin, ang):
        msg = Twist()
        msg.linear.x = lin
        msg.angular.z = ang
        self.cmd_pub.publish(msg)

    def get_yaw_from_quaternion(self, q):
        return tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
