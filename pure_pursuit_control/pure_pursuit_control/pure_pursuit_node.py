import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist
import math
import scipy.interpolate as si

lookahead_distance = 0.15
speed = 0.1

def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return yaw_z

def pure_pursuit(current_x, current_y, current_heading, path, index):
    global lookahead_distance
    closest_point = None
    v = speed
    for i in range(index, len(path)):
        x = path[i][0]
        y = path[i][1]
        distance = math.hypot(current_x - x, current_y - y)
        if lookahead_distance < distance:
            closest_point = (x, y)
            index = i
            break
    if closest_point is not None:
        target_heading = math.atan2(closest_point[1] - current_y, closest_point[0] - current_x)
        desired_steering_angle = target_heading - current_heading
    else:
        target_heading = math.atan2(path[-1][1] - current_y, path[-1][0] - current_x)
        desired_steering_angle = target_heading - current_heading
        index = len(path) - 1

    if desired_steering_angle > math.pi:
        desired_steering_angle -= 2 * math.pi
    elif desired_steering_angle < -math.pi:
        desired_steering_angle += 2 * math.pi

    if abs(desired_steering_angle) > math.pi / 6:
        sign = 1 if desired_steering_angle > 0 else -1
        desired_steering_angle = sign * math.pi / 4
        v = 0.0

    return v, desired_steering_angle, index

def bspline_planning(array, sn):
    try:
        array = np.array(array)
        x = array[:, 0]
        y = array[:, 1]
        N = 2
        t = range(len(x))
        x_tup = si.splrep(t, x, k=N)
        y_tup = si.splrep(t, y, k=N)

        x_list = list(x_tup)
        xl = x.tolist()
        x_list[1] = xl + [0.0, 0.0, 0.0, 0.0]

        y_list = list(y_tup)
        yl = y.tolist()
        y_list[1] = yl + [0.0, 0.0, 0.0, 0.0]

        ipl_t = np.linspace(0.0, len(x) - 1, sn)
        rx = si.splev(ipl_t, x_list)
        ry = si.splev(ipl_t, y_list)
        path = [(rx[i], ry[i]) for i in range(len(rx))]
    except:
        path = array
    return path

class NavigationControl(Node):
    def __init__(self):
        super().__init__('navigation_control')

        self.subscription_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.subscription_path = self.create_subscription(Path, '/global_plan', self.global_path_callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.timer = self.create_timer(0.01, self.timer_callback)

        self.path = []
        self.index = 0
        self.flag = 0  # 0: 대기, 1: 경로 수신됨

    def global_path_callback(self, msg):
        self.path = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        self.path = bspline_planning(self.path, len(self.path) * 5)
        self.index = 0
        self.flag = 1
        self.get_logger().info(f"경로 수신, 경로 길이: {len(self.path)}")

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = euler_from_quaternion(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )

    def timer_callback(self):
        if self.flag != 1 or len(self.path) == 0:
            return

        twist = Twist()
        v, w, self.index = pure_pursuit(self.x, self.y, self.yaw, self.path, self.index)
        twist.linear.x = v
        twist.angular.z = w

        if abs(self.x - self.path[-1][0]) < 0.05 and abs(self.y - self.path[-1][1]) < 0.05:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.flag = 0
            self.get_logger().info("경로 도착 완료.")

        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = NavigationControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

