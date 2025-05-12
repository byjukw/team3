import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped
from nav_msgs.msg import Path, OccupancyGrid, Odometry
from visualization_msgs.msg import Marker, MarkerArray
from global_planner.dstar.d_star_lite import DStarLite, Node as DStarNode
from scipy.interpolate import splprep, splev
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import numpy as np
import math

class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')

        # ───────── 파라미터 ─────────
        self.declare_parameter('inflation_radius', 0.2)
        self.declare_parameter('smooth_factor', 3.0)
        self.declare_parameter('max_curvature', 0.6)
        self.declare_parameter('publish_interval', 10)

        self.inflation_radius = self.get_parameter('inflation_radius').value
        self.base_smooth_factor = self.get_parameter('smooth_factor').value
        self.max_curvature = self.get_parameter('max_curvature').value
        self.publish_interval = int(self.get_parameter('publish_interval').value)

        # ───────── 구독 ─────────
        self.create_subscription(PointStamped, '/clicked_point',
                                 self.clicked_point_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        self.create_subscription(OccupancyGrid, '/map',
                                 self.map_callback, qos_profile=map_qos)

        # ───────── 발행 (⚠ DURABILITY = VOLATILE 로 통일) ─────────
        path_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE        # ← 수정 포인트
        )

        self.received_pub    = self.create_publisher(
            Path, '/global_path', path_qos)

        self.path_publisher   = self.received_pub
        
        self.marker_publisher = self.create_publisher(
            MarkerArray, '/waypoint_markers', 10)

        # ───────── 상태 변수 ─────────
        self.robot_position  = (0.0, 0.0)
        self.ox, self.oy     = [], []
        self.map_resolution  = 0.05
        self.map_origin      = (0.0, 0.0)
        self.map_received    = False
        self.odom_received   = False
        self.imu_received    = True   # IMU 미사용 시 True 로 둬서 패스
        self.last_goal       = None

    # ───── 이하 로직은 변경 없음 ─────
    def map_callback(self, msg):
        self.map_resolution = msg.info.resolution
        self.map_origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        width, height = msg.info.width, msg.info.height

        raw_obstacle_cells = [(x, y) for y in range(height) for x in range(width)
                              if msg.data[x + y * width] == 100]

        inflation_cells = int(self.inflation_radius / self.map_resolution + 0.5)
        inflated_cells = set()

        for (ox_cell, oy_cell) in raw_obstacle_cells:
            for dy in range(-inflation_cells, inflation_cells + 1):
                for dx in range(-inflation_cells, inflation_cells + 1):
                    nx, ny = ox_cell + dx, oy_cell + dy
                    if 0 <= nx < width and 0 <= ny < height:
                        inflated_cells.add((nx, ny))

        self.ox, self.oy = zip(*inflated_cells) if inflated_cells else ([], [])
        self.map_received = True

    def odom_callback(self, msg):
        world_x = msg.pose.pose.position.x
        world_y = msg.pose.pose.position.y
        grid_x  = int((world_x - self.map_origin[0]) / self.map_resolution)
        grid_y  = int((world_y - self.map_origin[1]) / self.map_resolution)
        self.robot_position = (grid_x, grid_y)
        self.odom_received  = True

    def clicked_point_callback(self, msg):
        self.get_logger().info(
            f"clicked at: ({msg.point.x:.2f}, {msg.point.y:.2f})")
        self.get_logger().info(
            f"[STATE] map:{self.map_received} odom:{self.odom_received}")

        if not (self.map_received and self.odom_received):
            return

        goal_x = int((msg.point.x - self.map_origin[0]) / self.map_resolution)
        goal_y = int((msg.point.y - self.map_origin[1]) / self.map_resolution)

        if self.last_goal == (goal_x, goal_y):
            return
        self.last_goal = (goal_x, goal_y)

        start_x, start_y = self.robot_position
        t_start = self.get_clock().now().nanoseconds

        waypoints = self.generate_dstar_path((start_x, start_y), (goal_x, goal_y))
        if not waypoints:
            self.get_logger().warn("No valid path found.")
            return

        if len(waypoints) > 300:
            waypoints = waypoints[::3]

        smoothed = self.smooth_path_with_curvature(waypoints)

        self.publish_waypoints(smoothed)
        self.publish_waypoint_markers(smoothed)

        t_end = self.get_clock().now().nanoseconds
        self.get_logger().info(f"{(t_end - t_start)/1e9:.3f} sec")

    def generate_dstar_path(self, start, goal):
        if not (self.ox and self.oy):
            return []
        try:
            planner = DStarLite(self.ox, self.oy)
            success, path_x, path_y = planner.main(
                DStarNode(start[0], start[1]),
                DStarNode(goal[0], goal[1]),
                spoofed_ox=[[]],
                spoofed_oy=[[]]
            )
        except Exception as e:
            self.get_logger().error(f"D* planner error: {e}")
            return []
        return list(zip(path_x, path_y)) if success else []

    def smooth_path_with_curvature(self, waypoints):
        if len(waypoints) < 3:
            return waypoints
        s_factor = self.base_smooth_factor
        for _ in range(15):
            x, y = zip(*waypoints)
            tck, _ = splprep([x, y], s=s_factor)
            u_fine = np.linspace(0, 1, len(waypoints) * 3)
            x_smooth, y_smooth = splev(u_fine, tck)
            smoothed = list(zip(x_smooth, y_smooth))
            real_waypts = [(wx * self.map_resolution + self.map_origin[0],
                            wy * self.map_resolution + self.map_origin[1])
                           for wx, wy in smoothed]
            if self.calculate_max_curvature(real_waypts) <= self.max_curvature + 1e-6:
                return smoothed
            s_factor *= 1.2
        self.get_logger().warn("Could not reduce curvature. Using last best.")
        return smoothed

    def publish_waypoints(self, waypoints):
        from rclpy.clock import Clock
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = Clock().now().to_msg()

        sub_waypoints = waypoints[1:]
        reduced = sub_waypoints[::self.publish_interval]
        if reduced and reduced[-1] != sub_waypoints[-1]:
            reduced.append(sub_waypoints[-1])
        elif not reduced and sub_waypoints:
            reduced = [sub_waypoints[-1]]

        for x, y in reduced:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = x * self.map_resolution + self.map_origin[0]
            pose.pose.position.y = y * self.map_resolution + self.map_origin[1]
            path_msg.poses.append(pose)

        self.path_publisher.publish(path_msg)

    def publish_waypoint_markers(self, waypoints):
        marker_array = MarkerArray()
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        self.marker_publisher.publish(marker_array)

        sub_waypoints = waypoints[30:]
        reduced = sub_waypoints[::self.publish_interval]
        if reduced and reduced[-1] != sub_waypoints[-1]:
            reduced.append(sub_waypoints[-1])
        elif not reduced and sub_waypoints:
            reduced = [sub_waypoints[-1]]

        for i, (x, y) in enumerate(reduced):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.ns = "waypoints"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x * self.map_resolution + self.map_origin[0]
            marker.pose.position.y = y * self.map_resolution + self.map_origin[1]
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker_array.markers.append(marker)

        self.marker_publisher.publish(marker_array)

    def calculate_max_curvature(self, real_waypoints):
        if len(real_waypoints) < 3:
            return 0.0
        max_curv = 0.0
        for i in range(1, len(real_waypoints) - 1):
            x1, y1 = real_waypoints[i - 1]
            x2, y2 = real_waypoints[i]
            x3, y3 = real_waypoints[i + 1]
            area = abs(x1*(y2 - y3) + x2*(y3 - y1) + x3*(y1 - y2)) / 2.0
            dist12 = math.hypot(x2 - x1, y2 - y1)
            dist23 = math.hypot(x3 - x2, y3 - y2)
            dist13 = math.hypot(x3 - x1, y3 - y1)
            denom = dist12 * dist23 * dist13
            if denom > 1e-9:
                curvature = (4.0 * area) / denom
                max_curv = max(max_curv, curvature)
        return max_curv

def main(args=None):
    rclpy.init(args=args)
    node = PathPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()