import os
import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory

class SteeringAnglePublisher(Node):
    def __init__(self):
        super().__init__('steering_angle_publisher')
        self.publisher_ = self.create_publisher(Float32, '/steering_angle', 10)

        # YOLO 모델 로드 (패키지 설치 경로에서 불러오기)
        model_path = os.path.join(get_package_share_directory('steering_yolo'), 'best.pt')
        self.model = YOLO(model_path)

        # 실시간 USB 카메라 연결 (번호는 환경에 따라 다를 수 있음)
        self.cap = cv2.VideoCapture(0)

        if not self.cap.isOpened():
            self.get_logger().error("❌ 카메라 연결 실패")
            exit()

        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("⚠️ 프레임을 읽지 못함")
            return

        frame_width = frame.shape[1]
        image_center_x = frame_width / 2
        max_angle = 17.5

        results = self.model(frame)
        for result in results[0].boxes:
            x_center, _, width, height = result.xywh[0].tolist()
            cls = int(result.cls[0].item())
            class_name = self.model.names[cls]

            if class_name == "Vest":
                error = image_center_x - x_center
                angle = (error / (frame_width / 2)) * max_angle

                msg = Float32()
                msg.data = angle
                self.publisher_.publish(msg)

                self.get_logger().info(f"[퍼블리시] 조향각: {angle:.2f}°")

                # 시각화용 바운딩 박스
                x1 = int(x_center - width / 2)
                y1 = int(result.xywh[0][1] - height / 2)
                x2 = int(x_center + width / 2)
                y2 = int(result.xywh[0][1] + height / 2)
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, f"{class_name} {angle:.1f}°", (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        cv2.imshow("YOLO Steering", frame)
        if cv2.waitKey(1) & 0xFF == 27:
            rclpy.shutdown()


def main():
    rclpy.init()
    node = SteeringAnglePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
