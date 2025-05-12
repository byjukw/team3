# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import Path, Odometry, OccupancyGrid
# from geometry_msgs.msg import PoseStamped
# from std_msgs.msg import Header
# from visualization_msgs.msg import Marker, MarkerArray
# import numpy as np
# from math import atan2, cos, sin
# import time
# from local_planner.hybrid_a_star import HybridAStar
# from scipy.ndimage import binary_dilation
# from scipy.interpolate import CubicHermiteSpline
# from nav_msgs.msg import OccupancyGrid, MapMetaData


# class LocalPlanner(Node):
#     def __init__(self):
#         super().__init__('local_planner')

#         self.declare_parameter('grid_size', 6.0)
#         self.declare_parameter('resolution', 0.05)

#         self.grid_size = self.get_parameter('grid_size').value
#         self.resolution = self.get_parameter('resolution').value

#         self.global_path = None
#         self.odom = None
#         self.costmap = None
#         self.costmap_origin_x = 0.0
#         self.costmap_origin_y = 0.0

#         self.create_subscription(Path, '/global_path', self.global_path_callback, 10)
#         self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
#         self.create_subscription(OccupancyGrid, '/local_costmap/costmap', self.costmap_callback, 10)

#         self.local_path_pub = self.create_publisher(Path, '/local_path', 10)
#         self.marker_pub = self.create_publisher(MarkerArray, '/local_path_markers', 10)
#         self.local_path_pub = self.create_publisher(Path, '/local_path', 10)
#         self.marker_pub = self.create_publisher(MarkerArray, '/local_path_markers', 10)
       
#         self.processed_costmap_pub = self.create_publisher(
#             OccupancyGrid,
#             '/our_local_costmap',
#             10
#         )
#         self.timer = self.create_timer(0.5, self.run_planner)

#     def global_path_callback(self, msg):
#         self.global_path = msg

#     def odom_callback(self, msg):
#         self.odom = msg

#     def costmap_callback(self, msg):
#         self.costmap_origin_x = msg.info.origin.position.x
#         self.costmap_origin_y = msg.info.origin.position.y
#         width = msg.info.width
#         height = msg.info.height
#         data = np.array(msg.data).reshape((height, width))
#         raw = np.where(data >= 100, 1, 0)
#         self.costmap = binary_dilation(raw, iterations=4).astype(np.uint8)
#         self.publish_processed_costmap()

#     def run_planner(self):
#         if not (self.global_path and self.odom and self.costmap is not None):
#             return

#         pose = self.odom.pose.pose
#         x = pose.position.x
#         y = pose.position.y
#         q = pose.orientation
#         yaw = atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y ** 2 + q.z ** 2))
#         start = (x - self.costmap_origin_x, y - self.costmap_origin_y, yaw)

#         goal_pose = self.find_safe_goal(x, y)
#         if not goal_pose:    
#             self.get_logger().warn("No valid safe local goal found.")
#             return

#         gx = goal_pose.position.x
#         gy = goal_pose.position.y
#         goal_tangent = self.estimate_tangent_from_global(gx, gy)
#         goal_yaw = atan2(goal_tangent[1], goal_tangent[0])
#         goal = (gx - self.costmap_origin_x, gy - self.costmap_origin_y, goal_yaw)

#         planner = HybridAStar(self.grid_size, self.resolution)
#         path = planner.plan(start, goal, self.costmap)
#         if not path:
#             self.get_logger().warn("Hybrid A* returned no path.")
#             return

#         smoothed_path = self.hermite_smooth_path(path)
#         self.publish_path_and_markers(smoothed_path)

#     def find_safe_goal(self, x, y, max_dist=2.0):
#         accumulated = 0.0
#         last_x, last_y = x, y
#         chosen_idx = -1
#         for i, pose_stamped in enumerate(self.global_path.poses):
#             wx, wy = pose_stamped.pose.position.x, pose_stamped.pose.position.y
#             dist = np.hypot(wx - last_x, wy - last_y)
#             accumulated += dist
#             last_x, last_y = wx, wy
#             mx = int((wx - self.costmap_origin_x) / self.resolution)
#             my = int((wy - self.costmap_origin_y) / self.resolution)
#             if 0 <= mx < self.costmap.shape[1] and 0 <= my < self.costmap.shape[0]:
#                 if self.costmap[my, mx] < 1 and accumulated >= max_dist:
#                     chosen_idx = i
#                     break
#             else:
#                 break
#         if chosen_idx != -1:
#             goal_idx = min(chosen_idx + 3, len(self.global_path.poses) - 1)
#             return self.global_path.poses[goal_idx].pose
#         return None
    
#     def publish_processed_costmap(self):
#                 # 2-1) MetaData 세팅
#                 meta = MapMetaData()
#                 meta.resolution = self.resolution
#                 meta.width  = self.costmap.shape[1]
#                 meta.height = self.costmap.shape[0]
#                 meta.origin.position.x = self.costmap_origin_x
#                 meta.origin.position.y = self.costmap_origin_y
#                 meta.origin.position.z = 0.0
#                 meta.origin.orientation.w = 1.0

#                 # 2-2) 데이터 Flatten (0,1 -> 0,100 for visibility)
#                 flat = (self.costmap * 100).astype(np.int8).flatten().tolist()

#                 grid = OccupancyGrid()
#                 grid.header.stamp = self.get_clock().now().to_msg()
#                 grid.header.frame_id = 'map'
#                 grid.info = meta
#                 grid.data = flat

#                 self.processed_costmap_pub.publish(grid)
#     def estimate_tangent_from_global(self, gx, gy):
#         poses = self.global_path.poses
#         for i in range(len(poses) - 1):
#             x1, y1 = poses[i].pose.position.x, poses[i].pose.position.y
#             x2, y2 = poses[i + 1].pose.position.x, poses[i + 1].pose.position.y
#             if np.hypot(gx - x1, gy - y1) < 0.2:
#                 dx, dy = x2 - x1, y2 - y1
#                 norm = np.hypot(dx, dy)
#                 return (dx / norm, dy / norm) if norm > 1e-6 else (1.0, 0.0)
#         return (1.0, 0.0)

#     def hermite_smooth_path(self, path):
#         if len(path) < 3:
#             return path

#         x = [p[0] for p in path]
#         y = [p[1] for p in path]
#         t = np.linspace(0, 1, len(x))
#         dx = np.gradient(x)
#         dy = np.gradient(y)

#         spline_x = CubicHermiteSpline(t, x, dx)
#         spline_y = CubicHermiteSpline(t, y, dy)

#         t_fine = np.linspace(0, 1, len(x) * 5)
#         x_smooth = spline_x(t_fine)
#         y_smooth = spline_y(t_fine)

#         # Downsample to avoid excessive waypoints
#         sampled = [(x_smooth[0], y_smooth[0], 0.0)]
#         for i in range(1, len(x_smooth)):
#             prev = sampled[-1]
#             dist = np.hypot(x_smooth[i] - prev[0], y_smooth[i] - prev[1])
#             if dist >= 0.3:
#                 sampled.append((x_smooth[i], y_smooth[i], 0.0))
#         return sampled

#     def publish_path_and_markers(self, path):
#         header = Header()
#         header.stamp = self.get_clock().now().to_msg()
#         header.frame_id = 'map'
#         path_msg = Path()
#         path_msg.header = header
#         for lx, ly, _ in path:
#             wx = lx + self.costmap_origin_x
#             wy = ly + self.costmap_origin_y
#             pose = PoseStamped()
#             pose.header = header
#             pose.pose.position.x = wx
#             pose.pose.position.y = wy
#             pose.pose.orientation.w = 1.0
#             path_msg.poses.append(pose)
#         self.local_path_pub.publish(path_msg)
#         self.publish_markers(path, header)

#     def publish_markers(self, path, header):
#         delete_all = Marker()
#         delete_all.action = Marker.DELETEALL
#         delete_all.header = header
#         delete_all.ns = "local_path"
#         self.marker_pub.publish(MarkerArray(markers=[delete_all]))
#         time.sleep(0.05)
#         marker_array = MarkerArray()
#         for i, (lx, ly, _) in enumerate(path):
#             wx = lx + self.costmap_origin_x
#             wy = ly + self.costmap_origin_y
#             marker = Marker()
#             marker.header = header
#             marker.ns = "local_path"
#             marker.id = i
#             marker.type = Marker.SPHERE
#             marker.action = Marker.ADD
#             marker.pose.position.x = wx
#             marker.pose.position.y = wy
#             marker.scale.x = marker.scale.y = marker.scale.z = 0.1
#             marker.color.r = 0.0
#             marker.color.g = 0.0
#             marker.color.b = 1.0
#             marker.color.a = 1.0
#             marker_array.markers.append(marker)
#         self.marker_pub.publish(marker_array)


# def main(args=None):
#     rclpy.init(args=args)
#     node = LocalPlanner()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry, OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from math import atan2, cos, sin
import time
from local_planner.hybrid_a_star import HybridAStar
from scipy.ndimage import binary_dilation
from scipy.interpolate import CubicHermiteSpline
from nav_msgs.msg import MapMetaData


class LocalPlanner(Node):
    def __init__(self):
        super().__init__('local_planner')

        self.declare_parameter('grid_size', 6.0)
        self.declare_parameter('resolution', 0.05)

        self.grid_size = self.get_parameter('grid_size').value
        self.resolution = self.get_parameter('resolution').value

        self.global_path = None
        self.odom = None
        self.costmap = None
        self.costmap_origin_x = 0.0
        self.costmap_origin_y = 0.0

        self.create_subscription(Path, '/global_path', self.global_path_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap', self.costmap_callback, 10)

        self.local_path_pub = self.create_publisher(Path, '/local_path', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/local_path_markers', 10)
        self.processed_costmap_pub = self.create_publisher(OccupancyGrid, '/our_local_costmap', 10)

        self.timer = self.create_timer(0.5, self.run_planner)

    def global_path_callback(self, msg):
        self.global_path = msg

    def odom_callback(self, msg):
        self.odom = msg

    def costmap_callback(self, msg):
        self.costmap_origin_x = msg.info.origin.position.x
        self.costmap_origin_y = msg.info.origin.position.y
        width = msg.info.width
        height = msg.info.height
        data = np.array(msg.data).reshape((height, width))
        raw = np.where(data >= 100, 1, 0)
        self.costmap = binary_dilation(raw, iterations=4).astype(np.uint8)
        self.publish_processed_costmap()

    def run_planner(self):
        if not (self.global_path and self.odom and self.costmap is not None):
            return

        pose = self.odom.pose.pose
        x = pose.position.x
        y = pose.position.y
        q = pose.orientation
        yaw = atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y ** 2 + q.z ** 2))
        start = (x - self.costmap_origin_x, y - self.costmap_origin_y, yaw)

        goal_pose = self.get_forward_target(x, y, distance=2.0)
        if not goal_pose:
            self.get_logger().warn("No valid safe local goal found.")
            return

        gx = goal_pose.position.x
        gy = goal_pose.position.y
        goal_tangent = self.estimate_tangent_from_global(gx, gy)
        goal_yaw = atan2(goal_tangent[1], goal_tangent[0])
        goal = (gx - self.costmap_origin_x, gy - self.costmap_origin_y, goal_yaw)

        planner = HybridAStar(self.grid_size, self.resolution)
        path = planner.plan(start, goal, self.costmap)
        if not path:
            self.get_logger().warn("Hybrid A* returned no path.")
            return

        smoothed_path = self.hermite_smooth_path(path)
        self.publish_path_and_markers(smoothed_path)

    def get_forward_target(self, x, y, distance=2.0):
        poses = self.global_path.poses

        # 1. 현재 위치에서 가장 가까운 전역 경로 인덱스 찾기
        min_idx = 0
        min_dist = float('inf')
        for i, pose in enumerate(poses):
            px, py = pose.pose.position.x, pose.pose.position.y
            d = np.hypot(x - px, y - py)
            if d < min_dist:
                min_idx = i
                min_dist = d

        # 2. 전역 경로 상에서 일정 거리만큼 떨어진 지점까지 누적 거리
        accumulated = 0.0
        last_x, last_y = x, y
        for i in range(min_idx, len(poses)):
            wx, wy = poses[i].pose.position.x, poses[i].pose.position.y
            dist = np.hypot(wx - last_x, wy - last_y)
            accumulated += dist
            last_x, last_y = wx, wy
            if accumulated >= distance:
                return poses[i].pose

        return poses[-1].pose

    def estimate_tangent_from_global(self, gx, gy):
        poses = self.global_path.poses
        for i in range(len(poses) - 1):
            x1, y1 = poses[i].pose.position.x, poses[i].pose.position.y
            x2, y2 = poses[i + 1].pose.position.x, poses[i + 1].pose.position.y
            if np.hypot(gx - x1, gy - y1) < 0.3:
                dx, dy = x2 - x1, y2 - y1
                norm = np.hypot(dx, dy)
                return (dx / norm, dy / norm) if norm > 1e-6 else (1.0, 0.0)
        return (1.0, 0.0)

    def hermite_smooth_path(self, path):
        if len(path) < 3:
            return path

        x = [p[0] for p in path]
        y = [p[1] for p in path]
        t = np.linspace(0, 1, len(x))
        dx = np.gradient(x)
        dy = np.gradient(y)

        spline_x = CubicHermiteSpline(t, x, dx)
        spline_y = CubicHermiteSpline(t, y, dy)

        t_fine = np.linspace(0, 1, len(x) * 5)
        x_smooth = spline_x(t_fine)
        y_smooth = spline_y(t_fine)

        sampled = [(x_smooth[0], y_smooth[0], 0.0)]
        for i in range(1, len(x_smooth)):
            prev = sampled[-1]
            dist = np.hypot(x_smooth[i] - prev[0], y_smooth[i] - prev[1])
            if dist >= 0.3:
                sampled.append((x_smooth[i], y_smooth[i], 0.0))
        return sampled

    def publish_path_and_markers(self, path):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'
        path_msg = Path()
        path_msg.header = header
        for lx, ly, _ in path:
            wx = lx + self.costmap_origin_x
            wy = ly + self.costmap_origin_y
            pose = PoseStamped()
            pose.header = header
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        self.local_path_pub.publish(path_msg)
        self.publish_markers(path, header)

    def publish_markers(self, path, header):
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        delete_all.header = header
        delete_all.ns = "local_path"
        self.marker_pub.publish(MarkerArray(markers=[delete_all]))
        time.sleep(0.05)
        marker_array = MarkerArray()
        for i, (lx, ly, _) in enumerate(path):
            wx = lx + self.costmap_origin_x
            wy = ly + self.costmap_origin_y
            marker = Marker()
            marker.header = header
            marker.ns = "local_path"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = wx
            marker.pose.position.y = wy
            marker.scale.x = marker.scale.y = marker.scale.z = 0.1
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 1.0
            marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)

    def publish_processed_costmap(self):
        meta = MapMetaData()
        meta.resolution = self.resolution
        meta.width = self.costmap.shape[1]
        meta.height = self.costmap.shape[0]
        meta.origin.position.x = self.costmap_origin_x
        meta.origin.position.y = self.costmap_origin_y
        meta.origin.position.z = 0.0
        meta.origin.orientation.w = 1.0

        flat = (self.costmap * 100).astype(np.int8).flatten().tolist()

        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = 'map'
        grid.info = meta
        grid.data = flat

        self.processed_costmap_pub.publish(grid)


def main(args=None):
    rclpy.init(args=args)
    node = LocalPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
