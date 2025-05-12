import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class DistancePIDController(Node):
    def __init__(self):
        super().__init__('distance_pid_controller')

        # 목표 거리
        self.target_distance = 1.5  

        # PID 게인 설정
        self.kp = 0.8   
        self.ki = 0.05  
        self.kd = 0.1   

        # PID 변수 초기화
        self.prev_error = 0.0
        self.integral = 0.0

        # 속도 제한
        self.max_speed = 1.33  
        self.min_speed = -1.33  

        # 거리 데이터 구독
        self.subscription = self.create_subscription(
            Float32,
            '/distance_data',  
            self.distance_callback,
            10)
        
        # 속도 퍼블리셔
        self.publisher = self.create_publisher(
            Twist, 
            '/cmd_vel',  
            10)
        
        self.get_logger().info("🚀 Distance PID Controller Initialized.")

    def distance_callback(self, msg):
        current_distance = msg.data  
        error = current_distance - self.target_distance  

        # PID 계산
        self.integral += error
        derivative = error - self.prev_error
        control = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error

        # 속도 제한
        control = max(min(control, self.max_speed), self.min_speed)

        # 속도 메시지 생성
        twist = Twist()
        twist.linear.x = control  
        twist.angular.z = 0.0  

        # 속도 퍼블리시
        self.publisher.publish(twist)
        self.get_logger().info(f"📏 거리: {current_distance:.2f}m, 🛠 속도: {control:.2f} m/s")

def main(args=None):
    rclpy.init(args=args)
    node = DistancePIDController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
