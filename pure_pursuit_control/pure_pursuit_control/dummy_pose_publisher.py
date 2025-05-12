import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math

class DummyPosePublisher(Node):
    def __init__(self):
        super().__init__('dummy_pose_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, '/position', 10)
        self.timer = self.create_timer(1.0, self.publish_pose)
        self.counter = 0  # 차량 위치 변화 카운터
        self.direction = 1  # 1: 오른쪽 이동, -1: 왼쪽 이동

    def publish_pose(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        # 차량 위치 계산 (경로 패턴과 유사하게 변경)
        self.x = self.counter * 0.5 * self.direction  # 직진 이동
        self.y = math.sin(self.counter * 0.3) * 2.5  # 자연스럽게 좌우 스윙

        msg.pose.position.x = self.x
        msg.pose.position.y = self.y

        self.publisher_.publish(msg)
        self.get_logger().info(f"Published /position: x={self.x:.2f}, y={self.y:.2f}")

        self.counter += 1

        # 🔄 **10초마다 방향 전환**
        if self.counter % 10 == 0:
            self.direction *= -1  # 방향 바꾸기

def main(args=None):
    rclpy.init(args=args)
    node = DummyPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

