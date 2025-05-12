import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32

class JoystickToSteering(Node):
    def __init__(self):
        super().__init__('joystick_to_steering')
        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.publisher = self.create_publisher(Float32, '/joystick_steering_angle', 10)

    def joy_callback(self, msg):
        axis_value = msg.axes[0]  # ← 축 0번으로 수정!
        angle = -axis_value * 17.5  # -17.5 ~ +17.5도 범위로 조향각 변환
        self.get_logger().info(f'조향각: {angle:.2f}')
        self.publisher.publish(Float32(data=angle))

def main(args=None):
    rclpy.init(args=args)
    node = JoystickToSteering()
    rclpy.spin(node)
    rclpy.shutdown()
