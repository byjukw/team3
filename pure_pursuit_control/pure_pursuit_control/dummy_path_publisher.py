import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math

class DummyPathPublisher(Node):
    def __init__(self):
        super().__init__('dummy_path_publisher')
        self.publisher_ = self.create_publisher(Path, '/local_path', 10)
        self.timer = self.create_timer(1.0, self.publish_path)  # 1초마다 실행
        self.counter = 0  # 경로 변경을 위한 카운터
        self.direction = 1  # 1: 오른쪽 이동, -1: 왼쪽 이동

    def publish_path(self):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        # 🔄 **방향에 따라 오른쪽 / 왼쪽 이동을 조정**
        base_x = self.counter * 0.5 * self.direction  # 방향에 따라 x값 증가/감소
        base_y = math.sin(self.counter * 0.3) * 2.5  # 더 부드럽고 자연스럽게 이동

        waypoints = [
            (base_x, base_y), (base_x + 1.0, base_y + 0.2), (base_x + 2.0, base_y + 0.4),
            (base_x + 3.0, base_y + 0.6), (base_x + 4.0, base_y + 0.8)
        ]

        for wp in waypoints:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = wp[0]
            pose.pose.position.y = wp[1]
            msg.poses.append(pose)

        self.publisher_.publish(msg)
        self.get_logger().info(f"Published /local_path with base_x={base_x}, base_y={base_y}, direction={self.direction}")

        # 🔄 **10초마다 이동 방향을 바꾼다**
        self.counter += 1
        if self.counter % 10 == 0:  # 10초마다 전환
            self.direction *= -1  # 방향 바꾸기

def main(args=None):
    rclpy.init(args=args)
    node = DummyPathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

