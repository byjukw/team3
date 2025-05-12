#!/usr/bin/env python3
import time
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped, PointStamped, TransformStamped
from nav_msgs.msg import Path, OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from global_planner.dstar.d_star_lite import DStarLite, Node as DStarNode
from scipy.interpolate import splprep, splev
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import tf2_ros

class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')

        # ───────── 파라미터 ─────────
        self.declare_parameter('inflation_radius', 0.2)
        self.declare_parameter('smooth_factor', 3.0)
        self.declare_parameter('max_curvature', 0.6)
        self.declare_parameter('publish_interval', 10)

        self.inflation_radius   = self.get_parameter('inflation_radius').value
        self.base_smooth_factor = self.get_parameter('smooth_factor').value
        self.max_curvature      = self.get_parameter('max_curvature').value
        self.publish_interval   = int(self.get_parameter('publish_interval').value)

        # ───────── TF2 Buffer & Listener ─────────
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ───────── 구독 ─────────
        self.create_subscription(PointStamped, '/clicked_point',
                                 self.clicked_point_callback, 10)
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        self.create_subscription(OccupancyGrid, '/map',
                                 self.map_callback, qos_profile=map_qos)

        # ───────── 발행 (⚠ DURABILITY = VOLATILE) ─────────
        path_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )
        self.path_publisher   = self.create_publisher(Path, '/global_path', path_qos)
        self.marker_publisher = self.create_publisher(MarkerArray, '/waypoint_markers', 10)

        # ───────── 상태 변수 ─────────
        self.robot_position = (0, 0)
        self.ox, self.oy    = [], []
        self.map_resolution = 0.05
        self.map_origin     = (0.0, 0.0)
        self.map_received   = False
        self.odom_received  = False
        self.last_goal      = None

    def map_callback(self, msg: OccupancyGrid):
        self.map_resolution = msg.info.resolution
        self.map_origin = (msg.info.origin.position.x,
                           msg.info.origin.position.y)
        width, height = msg.info.width, msg.info.height

        raw = [(x, y)
               for y in range(height) for x in range(width)
               if msg.data[x + y * width] == 100]

        inflate = int(self.inflation_radius / self.map_resolution + 0.5)
        cells = set()
        for ox, oy in raw:
            for dy in range(-inflate, inflate + 1):
                for dx in range(-inflate, inflate + 1):
                    nx, ny = ox + dx, oy + dy
                    if 0 <= nx < width and 0 <= ny < height:
                        cells.add((nx, ny))

        if cells:
            self.ox, self.oy = zip(*cells)
        else:
            self.ox, self.oy = [], []
        self.map_received = True

    def update_robot_position(self):
        # odom -> base_footprint lookup, 최대 5회 재시도
        for _ in range(5):
            try:
                t: TransformStamped = self.tf_buffer.lookup_transform(
                    'odom', 'base_footprint', Time())
                wx = t.transform.translation.x
                wy = t.transform.translation.y
                gx = int((wx - self.map_origin[0]) / self.map_resolution)
                gy = int((wy - self.map_origin[1]) / self.map_resolution)
                self.robot_position = (gx, gy)
                self.odom_received = True
                return
            except (tf2_ros.LookupException,
                    tf2_ros.ExtrapolationException,
                    tf2_ros.ConnectivityException):
                time.sleep(0.05)
        self.get_logger().warn("TF lookup failed: odom→base_footprint")
        self.odom_received = False

    def clicked_point_callback(self, msg: PointStamped):
        # TF에서 위치 갱신
        self.update_robot_position()

        self.get_logger().info(f"clicked at: ({msg.point.x:.2f}, {msg.point.y:.2f})")
        self.get_logger().info(f"[STATE] map:{self.map_received} odom:{self.odom_received}")
        if not (self.map_received and self.odom_received):
            return

        goal_x = int((msg.point.x - self.map_origin[0]) / self.map_resolution)
        goal_y = int((msg.point.y - self.map_origin[1]) / self.map_resolution)
        if (goal_x, goal_y) == self.last_goal:
            return
        self.last_goal = (goal_x, goal_y)

        waypoints = self.generate_dstar_path(self.robot_position, (goal_x, goal_y))
        if not waypoints:
            self.get_logger().warn("No valid path found.")
            return

        if len(waypoints) > 300:
            waypoints = waypoints[::3]
        smoothed = self.smooth_path_with_curvature(waypoints)

        self.publish_waypoints(smoothed)
        self.publish_waypoint_markers(smoothed)

    def generate_dstar_path(self, start, goal):
        if not (self.ox and self.oy):
            return []
        try:
            planner = DStarLite(self.ox, self.oy)
            success, px, py = planner.main(
                DStarNode(start[0], start[1]),
                DStarNode(goal[0], goal[1]),
                spoofed_ox=[[]], spoofed_oy=[[]]
            )
        except Exception as e:
            self.get_logger().error(f"D* planner error: {e}")
            return []
        return list(zip(px, py)) if success else []

    def smooth_path_with_curvature(self, waypoints):
        if len(waypoints) < 3:
            return waypoints
        s = self.base_smooth_factor
        for _ in range(15):
            x, y = zip(*waypoints)
            tck, _ = splprep([x, y], s=s)
            u_fine = np.linspace(0, 1, len(waypoints) * 3)
            xs, ys = splev(u_fine, tck)
            sm = list(zip(xs, ys))
            real = [(
                wx * self.map_resolution + self.map_origin[0],
                wy * self.map_resolution + self.map_origin[1]
            ) for wx, wy in sm]
            if self.calculate_max_curvature(real) <= self.max_curvature + 1e-6:
                return sm
            s *= 1.2
        self.get_logger().warn("Could not reduce curvature. Using last best.")
        return sm

    def publish_waypoints(self, waypoints):
        from rclpy.clock import Clock
        msg = Path()
        msg.header.frame_id = 'map'
        msg.header.stamp = Clock().now().to_msg()

        pts = waypoints[1:]
        red = pts[::self.publish_interval]
        if red and red[-1] != pts[-1]:
            red.append(pts[-1])
        elif not red and pts:
            red = [pts[-1]]

        for wx, wy in red:
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.pose.position.x = wx * self.map_resolution + self.map_origin[0]
            ps.pose.position.y = wy * self.map_resolution + self.map_origin[1]
            msg.poses.append(ps)

        self.path_publisher.publish(msg)

    def publish_waypoint_markers(self, waypoints):
        array = MarkerArray()
        delete = Marker()
        delete.action = Marker.DELETEALL
        array.markers.append(delete)
        self.marker_publisher.publish(array)

        pts = waypoints[30:]
        red = pts[::self.publish_interval]
        if red and red[-1] != pts[-1]:
            red.append(pts[-1])
        elif not red and pts:
            red = [pts[-1]]

        array = MarkerArray()
        for i, (wx, wy) in enumerate(red):
            m = Marker()
            m.header.frame_id = 'map'
            m.ns = 'waypoints'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = wx * self.map_resolution + self.map_origin[0]
            m.pose.position.y = wy * self.map_resolution + self.map_origin[1]
            m.scale.x = m.scale.y = m.scale.z = 0.1
            m.color.a = 1.0
            m.color.r = 1.0
            array.markers.append(m)
        self.marker_publisher.publish(array)

    def calculate_max_curvature(self, real):
        if len(real) < 3:
            return 0.0
        max_curv = 0.0
        for i in range(1, len(real)-1):
            x1, y1 = real[i-1]
            x2, y2 = real[i]
            x3, y3 = real[i+1]
            area = abs(x1*(y2-y3) + x2*(y3-y1) + x3*(y1-y2)) / 2.0
            d12 = math.hypot(x2-x1, y2-y1)
            d23 = math.hypot(x3-x2, y3-y2)
            d13 = math.hypot(x3-x1, y3-y1)
            denom = d12 * d23 * d13
            if denom > 1e-9:
                curv = (4.0 * area) / denom
                max_curv = max(max_curv, curv)
        return max_curv

def main(args=None):
    rclpy.init(args=args)
    node = PathPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
