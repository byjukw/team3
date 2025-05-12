import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Float32
import math

class PurePursuitController(Node):
    def __init__(self):
        """ Pure Pursuit 알고리즘을 실행하는 ROS 2 노드 """
        super().__init__('pure_pursuit_controller')

        # 📌 웨이포인트 경로 구독 (/local_path)
        self.subscription_path = self.create_subscription(Path, '/local_path', self.path_callback, 10)

        # 📌 현재 차량 위치 구독 (/position)
        self.subscription_pose = self.create_subscription(PoseStamped, '/position', self.pose_callback, 10)

        # 📌 트래킹 대상과의 거리 구독 (/tracking_distance)
        self.subscription_tracking = self.create_subscription(Float32, '/tracking_distance', self.tracking_callback, 10)

        # 📌 조향각 퍼블리시 (/steering_angle)
        self.steer_pub = self.create_publisher(Float32, '/steering_angle', 10)

        # 🔹 Look-Ahead Distance 범위 (최소~최대)
        self.min_lookahead_distance = 0.7  # 최소 0.7m
        self.max_lookahead_distance = 2.5  # 최대 2.5m

        # 🔹 차량 파라미터
        self.L = 0.5  # 휠베이스 (차량 앞바퀴-뒷바퀴 거리)
        self.max_steering_angle = math.radians(17.5)  # 최대 조향각 ±17.5° (0.305 rad)
        self.max_steering_rate = math.radians(30)  # 최대 조향각 변화율 30°/s (0.52 rad/s)

        # 🔹 기본 상태값
        self.tracking_distance = 1.5  # 기본값 (초기 설정)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.previous_steering_angle = 0.0  # 이전 조향각 저장용
        self.path = []  # 웨이포인트 저장

    def tracking_callback(self, msg):
        """ 📌 트래킹 대상과 차량 사이의 거리 정보 업데이트 """
        self.tracking_distance = msg.data

    def path_callback(self, msg):
        """ 📌 /local_path에서 웨이포인트를 받아서 저장 """
        self.path = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]

    def pose_callback(self, msg):
        """ 📌 /position에서 현재 차량 위치와 방향(Yaw) 업데이트 """
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        self.current_yaw = self.quaternion_to_yaw(msg.pose.orientation)

        # 🔹 트래킹 거리 기반 Look-Ahead Distance 조정
        self.lookahead_distance = self.compute_dynamic_lookahead_distance()

        # 🚗 Pure Pursuit 알고리즘 실행 (조향각 계산 및 퍼블리시)
        self.control_loop()

    def compute_dynamic_lookahead_distance(self):
        """ 📌 트래킹 거리 기반 Look-Ahead Distance 조정 """
        # 🔹 기본적으로 트래킹 거리보다 조금 여유를 두고 설정
        lookahead = self.tracking_distance * 1.2  # 트래킹 거리의 1.2배

        # 🔹 LAD 값이 최소~최대 범위를 벗어나지 않도록 보정
        return max(self.min_lookahead_distance, min(self.max_lookahead_distance, lookahead))

    def limit_steering_rate(self, new_steering_angle):
        """ 📌 조향각 변화율 제한 (Δθ ≤ 30°/s) """
        dt = 0.1  # 제어 주기 (100ms)
        max_delta_theta = self.max_steering_rate * dt  # 최대 변화 가능 각도

        # 🔹 현재 조향각과 이전 조향각 차이 계산
        delta_theta = new_steering_angle - self.previous_steering_angle

        # 🔹 변화율이 너무 크다면 제한 (30°/s)
        if abs(delta_theta) > max_delta_theta:
            new_steering_angle = self.previous_steering_angle + math.copysign(max_delta_theta, delta_theta)

        # 🔹 이전 조향각 업데이트
        self.previous_steering_angle = new_steering_angle

        return new_steering_angle

    def control_loop(self):
        """ 📌 Pure Pursuit 알고리즘 실행 및 조향각 퍼블리시 """
        if not self.path:
            return

        # 🔹 Look-Ahead Distance 내에서 목표 웨이포인트 찾기
        target_wp = self.find_target_waypoint()
        if target_wp:
            # 🔹 목표 웨이포인트로부터 조향각 계산
            steering_angle = self.compute_steering_angle(target_wp[0], target_wp[1])

            # 🔹 조향각 변화율 제한 적용 (급격한 조향 방지)
            steering_angle = self.limit_steering_rate(steering_angle)

            # 🔹 조향각 퍼블리시
            steer_msg = Float32()
            steer_msg.data = steering_angle
            self.steer_pub.publish(steer_msg)

    def find_target_waypoint(self):
        """ 📌 Look-Ahead Distance 내에 있는 목표 웨이포인트 찾기 """
        if not self.path:
            return None
        for (x, y) in self.path:
            distance = math.sqrt((x - self.current_x) ** 2 + (y - self.current_y) ** 2)
            if distance >= self.lookahead_distance:
                return (x, y)
        return self.path[-1] if self.path else None

    def compute_steering_angle(self, target_x, target_y):
        """ 📌 Pure Pursuit 알고리즘을 적용하여 조향각 계산 """
        angle_to_target = math.atan2(target_y - self.current_y, target_x - self.current_x)
        alpha = angle_to_target - self.current_yaw
        steering_angle = math.atan2(2 * self.L * math.sin(alpha), self.lookahead_distance)

        # 🔹 최대 조향각 제한 (±17.5°)
        return max(-self.max_steering_angle, min(self.max_steering_angle, steering_angle))

    def quaternion_to_yaw(self, orientation):
        """ 📌 쿼터니언을 Yaw 각도로 변환 """
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y**2 + orientation.z**2)
        return math.atan2(siny_cosp, cosy_cosp)

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

