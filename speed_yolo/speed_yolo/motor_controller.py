import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from rclpy.executors import MultiThreadedExecutor
import time
from collections import deque

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')

        self.subscription = self.create_subscription(
            Float32, 'bounding_box_size', self.listener_callback, 50
        )
        self.pwm_publisher = self.create_publisher(Float32, 'motor_pwm', 10)

        self.target_bbox = 50000
        self.kp = 0.0002
        self.ki = 0.000005
        self.kd = 0.0002

        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = None

        self.min_pwm = 0
        self.max_pwm = 30
        self.stop_box_size = 50000

        self.last_bbox_size = None
        self.last_pwm_sent = None
        self.last_update_time = time.time()

        self.no_detection_timeout = 1.5

        self.ignore_small_bbox_until = None
        self.min_valid_bbox_area = 20000
        self.sudden_drop_ratio = 0.5
        self.recovery_timeout = 1.0
        self.last_valid_bbox = None

        self.bbox_history = deque(maxlen=5)
        self.zero_count = 0
        self.recovery_from_zero = False
        self.same_bbox_count = 0
        self.same_bbox_threshold = 10
        self.same_bbox_value = None

    def listener_callback(self, msg):
        now = time.time()
        self.get_logger().debug(f"[RECV] bbox: {msg.data:.1f}")

        if msg.data == 0.0:
            self.zero_count += 1
            if self.zero_count >= 5:
                self.get_logger().info("📭 bbox=0.0 연속 5회 → No detection")
                self.last_bbox_size = 0.0
                self.recovery_from_zero = True
                self.control_motor()
            return
        else:
            self.zero_count = 0

        if self.same_bbox_value is not None and abs(msg.data - self.same_bbox_value) < 5000:
            self.same_bbox_count += 1
        else:
            self.same_bbox_count = 1
            self.same_bbox_value = msg.data

        if self.same_bbox_count >= self.same_bbox_threshold:
            self.get_logger().info(f"✅ 동일 bbox {self.same_bbox_count}회 → 정상값 인식: {msg.data:.0f}")
            self.last_valid_bbox = msg.data

        if self.last_bbox_size and msg.data < self.last_bbox_size * self.sudden_drop_ratio and msg.data < self.min_valid_bbox_area:
            self.get_logger().warn(f"⚠️ bbox 축소 감지: {self.last_bbox_size:.0f} → {msg.data:.0f} → 일시 무시")
            self.ignore_small_bbox_until = now + self.recovery_timeout
            return

        if self.ignore_small_bbox_until and now < self.ignore_small_bbox_until:
            if msg.data < self.min_valid_bbox_area:
                self.get_logger().debug("⏳ 작은 bbox 무시 중...")
                return
            else:
                self.get_logger().info(f"✅ bbox 복구 감지: {msg.data:.0f} → 사용 재개")
                self.ignore_small_bbox_until = None

        if self.last_bbox_size is not None and self.last_bbox_size > 1000:
            ratio = abs(msg.data - self.last_bbox_size) / (self.last_bbox_size + 1e-5)
            diff = abs(msg.data - self.last_bbox_size)
            if not self.recovery_from_zero and ratio > 0.3 and diff > 8000:
                self.get_logger().warn(f"⚠️ bbox 튐 감지: {self.last_bbox_size:.1f} → {msg.data:.1f} → 무시됨")
                return

        self.recovery_from_zero = False
        self.bbox_history.append(msg.data)
        smoothed_bbox = sum(self.bbox_history) / len(self.bbox_history)

        if self.last_valid_bbox is None or smoothed_bbox > self.last_valid_bbox * 1.2:
            self.last_valid_bbox = smoothed_bbox
            self.get_logger().info(f"✅ 복구 후 정상 bbox 사용: {smoothed_bbox:.0f}")

        self.last_update_time = now
        self.last_bbox_size = smoothed_bbox
        self.control_motor()

    def control_motor(self):
        current_time = time.time()

        if self.last_bbox_size is None or self.last_bbox_size == 0.0:
            pwm_value = 0
        elif self.last_bbox_size >= self.stop_box_size:
            pwm_value = 0
        else:
            error = self.target_bbox - self.last_bbox_size
            dt = current_time - self.last_time if self.last_time else 0.05

            self.integral += error * dt
            derivative = (error - self.last_error) / dt if dt > 0 else 0

            pwm_value = self.kp * error + self.ki * self.integral + self.kd * derivative
            pwm_value = int(max(self.min_pwm, min(self.max_pwm, pwm_value)))

            self.last_error = error
            self.last_time = current_time

        max_change = 3
        if self.last_pwm_sent is None:
            smoothed_pwm = pwm_value
        else:
            delta = pwm_value - self.last_pwm_sent
            smoothed_pwm = self.last_pwm_sent + max_change * (1 if delta > 0 else -1) if abs(delta) > max_change else pwm_value

        if smoothed_pwm != self.last_pwm_sent:
            self.get_logger().info(f"🌀 PWM 제어 → {smoothed_pwm} (bbox: {self.last_bbox_size:.1f})")
            self.last_pwm_sent = smoothed_pwm

            pwm_msg = Float32()
            pwm_msg.data = float(smoothed_pwm)
            self.pwm_publisher.publish(pwm_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MotorController()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.05)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

