import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random

class DummyTrackingDistancePublisher(Node):
    def __init__(self):
        """ 트래킹 대상과의 거리 정보를 생성하는 ROS 2 노드 """
        super().__init__('dummy_tracking_distance_publisher')
        self.publisher_ = self.create_publisher(Float32, '/tracking_distance', 10)
        self.timer = self.create_timer(1.0, self.publish_tracking_distance)  # 1초마다 실행

    def publish_tracking_distance(self):
        """ 📌 트래킹 거리 임의 생성 (1.0m ~ 2.0m 사이 랜덤) """
        tracking_distance = random.uniform(1.0, 2.0)  # 1.0 ~ 2.0m 랜덤 생성
        msg = Float32()
        msg.data = tracking_distance
        self.publisher_.publish(msg)

        self.get_logger().info(f"Published /tracking_distance: {tracking_distance:.2f}m")

def main(args=None):
    rclpy.init(args=args)
    node = DummyTrackingDistancePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

