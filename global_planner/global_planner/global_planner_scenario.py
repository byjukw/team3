#!/usr/bin/env python3
# coding: utf-8

# 필요한 라이브러리 임포트
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path, OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from global_planner.dstar.d_star_lite_final import DStarLite, Node as DStarNode
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from scipy.interpolate import splprep, splev
import numpy as np
import math
from nav_msgs.msg import Odometry

class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')
        
        # 파라미터 설정
        self.declare_parameter('inflation_radius', 0.3)
        self.declare_parameter('smooth_factor', 3.0)
        self.declare_parameter('max_curvature', 0.585)
        self.declare_parameter('publish_interval', 10)

        self.inflation_radius = self.get_parameter('inflation_radius').value
        self.base_smooth_factor = self.get_parameter('smooth_factor').value
        self.max_curvature = self.get_parameter('max_curvature').value
        self.publish_interval = int(self.get_parameter('publish_interval').value)

        # 초기 위치 설정
        self.start_world_x = 0.0
        self.start_world_y = 0.0
        
        # 토픽 구독 설정
        self.create_subscription(
            Odometry, '/imu/data',
            self.imu_callback, 10)
            
        self.create_subscription(
            PointStamped, '/clicked_point',
            self.clicked_point_callback, 10)

        # 맵 구독 설정
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        self.create_subscription(
            OccupancyGrid, '/map',
            self.map_callback, qos_profile=map_qos)

        # 발행자 설정
        path_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )
        self.plan_pub = self.create_publisher(Path, '/global_path', path_qos)
        self.marker_pub = self.create_publisher(MarkerArray, '/waypoint_markers', 10)

        # 상태 변수 초기화
        self.map_received = False
        self.ox = []
        self.oy = []
        self.map_resolution = 0.05
        self.map_origin = (0.0, 0.0)
        self.last_goal = None

    def imu_callback(self, msg: Odometry):
        self.start_world_x = msg.pose.pose.position.x
        self.start_world_y = msg.pose.pose.position.y

    def map_callback(self, msg: OccupancyGrid):
        # 맵 정보 저장
        self.map_resolution = msg.info.resolution
        self.map_origin = (msg.info.origin.position.x,
                          msg.info.origin.position.y)
        self.map_width = msg.info.width
        self.map_height = msg.info.height

        # 장애물 정보 처리
        occupied_thresh = int(0.65 * 100)
        raw = []
        for y in range(self.map_height):
            base = y * self.map_width
            for x in range(self.map_width):
                v = msg.data[base + x]
                if v >= occupied_thresh or v == -1:
                    raw.append((x, y))

        # 장애물 팽창
        inf_cells = int(self.inflation_radius / self.map_resolution + 0.5)
        inflated = set()
        for (ox, oy) in raw:
            for dy in range(-inf_cells, inf_cells+1):
                for dx in range(-inf_cells, inf_cells+1):
                    nx, ny = ox+dx, oy+dy
                    if 0 <= nx < self.map_width and 0 <= ny < self.map_height:
                        inflated.add((nx, ny))
        self.ox, self.oy = zip(*inflated) if inflated else ([], [])
        self.map_received = True

    def clicked_point_callback(self, msg: PointStamped):
        # 맵 수신 확인
        if not self.map_received:
            self.get_logger().warn("Waiting for map...")
            return

        # 목표점 변환 및 검증
        gx = int((msg.point.x - self.map_origin[0]) / self.map_resolution)
        gy = int((msg.point.y - self.map_origin[1]) / self.map_resolution)
        
        self.get_logger().info(
            f"Clicked world ({msg.point.x:.2f}, {msg.point.y:.2f}) → grid ({gx}, {gy})"
        )
        
        if not (0 <= gx < self.map_width and 0 <= gy < self.map_height):
            self.get_logger().error(
                f"Goal out of bounds: ({gx},{gy}), map {self.map_width}×{self.map_height}"
            )
            return
            
        goal = (gx, gy)
        if goal == self.last_goal:
            return
        self.last_goal = goal

        # 시작점 설정 및 경로 계획
        if self.start_world_x is None or self.start_world_y is None:
            self.get_logger().warn("No IMU position yet; cannot plan.")
            return
            
        sx = int((self.start_world_x - self.map_origin[0]) / self.map_resolution)
        sy = int((self.start_world_y - self.map_origin[1]) / self.map_resolution)
        start = (sx, sy)
        
        self.get_logger().info(
            f"Start world ({self.start_world_x:.2f}, {self.start_world_y:.2f}) → "
            f"grid ({sx}, {sy})"
        )

        # 경로 계획 및 처리
        t0 = self.get_clock().now().nanoseconds
        raw_path = self.generate_dstar_path(start, goal)
        t1 = self.get_clock().now().nanoseconds

        if not raw_path:
            self.get_logger().warn("No path found.")
            return

        smoothed = self.smooth_path_with_curvature(raw_path)
        self.publish_plan(smoothed)
        self.publish_markers(smoothed)

        # 결과 로깅
        duration_ms = (t1 - t0) / 1e6
        num_wp = len(smoothed)
        real_pts = [(x*self.map_resolution + self.map_origin[0],
                    y*self.map_resolution + self.map_origin[1])
                   for x, y in smoothed]
        length_m = sum(
            math.hypot(real_pts[i+1][0]-real_pts[i][0],
                      real_pts[i+1][1]-real_pts[i][1])
            for i in range(len(real_pts)-1)
        )
        self.get_logger().info(
            f"Planning done: {duration_ms:.2f} ms, waypoints: {num_wp}, length: {length_m:.2f} m"
        )

    def generate_dstar_path(self, start, goal):
        try:
            planner = DStarLite(
                self.ox,
                self.oy,
                width=self.map_width,
                height=self.map_height
            )
            # spoofed_ox, spoofed_oy 인자 제거
            success, path_x, path_y = planner.main(
                DStarNode(start[0], start[1]),
                DStarNode(goal[0],  goal[1])
            )
        except Exception as e:
            self.get_logger().error(f"D* planner error: {e}")
            return []

        if not success:
            return []

        return list(zip(path_x, path_y))

    def smooth_path_with_curvature(self, pts):
        if len(pts) < 3:
            return pts
        s_factor = self.base_smooth_factor
        for _ in range(15):
            x, y = zip(*pts)
            tck, _ = splprep([x, y], s=s_factor)
            u = np.linspace(0, 1, len(pts) * 3)
            xs, ys = splev(u, tck)
            smoothed = list(zip(xs, ys))

            real_pts = [(x * self.map_resolution + self.map_origin[0],
                         y * self.map_resolution + self.map_origin[1])
                        for x, y in smoothed]

            if self.calculate_max_curvature(real_pts) <= self.max_curvature + 1e-6:
                return smoothed
            s_factor *= 1.2 #1.2
        self.get_logger().warn("Could not reduce curvature. Using last best.")
        return smoothed

    def calculate_max_curvature(self, pts):
        if len(pts) < 3:
            return 0.0
        max_curv = 0.0
        for i in range(1, len(pts) - 1):
            x1, y1 = pts[i - 1]
            x2, y2 = pts[i]
            x3, y3 = pts[i + 1]
            area = abs(x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)) / 2.0
            d12 = math.hypot(x2 - x1, y2 - y1)
            d23 = math.hypot(x3 - x2, y3 - y2)
            d13 = math.hypot(x3 - x1, y3 - y1)
            denom = d12 * d23 * d13
            if denom > 1e-9:
                curvature = (4.0 * area) / denom
                max_curv = max(max_curv, curvature)
        return max_curv

    def publish_plan(self, pts):
        from rclpy.clock import Clock
        msg = Path()
        msg.header.frame_id = 'map'
        msg.header.stamp = Clock().now().to_msg()
        pts2 = pts[1::self.publish_interval] + [pts[-1]]
        for x, y in pts2:
            p = PoseStamped()
            p.header = msg.header
            p.pose.position.x = x*self.map_resolution + self.map_origin[0]
            p.pose.position.y = y*self.map_resolution + self.map_origin[1]
            msg.poses.append(p)
        self.plan_pub.publish(msg)

    def publish_markers(self, pts):
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        pts2 = pts[1::self.publish_interval] + [pts[-1]]
        for i, (x, y) in enumerate(pts2):
            m = Marker()
            m.header.frame_id = 'map'
            m.ns = 'wp'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.scale.x = m.scale.y = m.scale.z = 0.1
            m.color.a, m.color.r, m.color.g, m.color.b = (1.0,1.0,0.0,0.0)
            m.pose.position.x = x*self.map_resolution + self.map_origin[0]
            m.pose.position.y = y*self.map_resolution + self.map_origin[1]
            markers.markers.append(m)
        self.marker_pub.publish(markers)

def main(args=None):
    rclpy.init(args=args)
    node = PathPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
