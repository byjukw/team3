import os
import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import torch
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory

BOUNDING_BOX_COLOR = (0, 255, 255)
STEERING_BOX_COLOR = (0, 255, 0)

class YoloVideoPublisher(Node):
    def __init__(self):
        super().__init__('yolo_video_publisher')

        # 퍼블리셔 설정
        self.bbox_publisher = self.create_publisher(Float32, 'bounding_box_size', 10)
        self.steering_publisher = self.create_publisher(Float32, '/steering_angle', 10)

        # YOLO 모델 로드
        model_path = os.path.join(get_package_share_directory('steering_yolo'), 'best.pt')
        device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.model = YOLO(model_path).to(device)

        # 카메라 열기 (0번,1,2)
        self.cap = cv2.VideoCapture(2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)
        if not self.cap.isOpened():
            self.get_logger().error("❌ 웹캠 열 수 없음")
            exit(1)

        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("⚠️ 프레임을 읽을 수 없음")
            return

        results = self.model(frame, verbose=False)
        frame_width = frame.shape[1]
        image_center_x = frame_width / 2
        max_angle = 17.5

        bbox_size = 0.0
        angle = None

        for result in results:
            boxes = result.boxes
            if boxes is None or len(boxes) == 0:
                continue

            for i in range(len(boxes)):
                cls_id = int(boxes.cls[i].item())
                class_name = self.model.names[cls_id]
                x1, y1, x2, y2 = boxes.xyxy[i].cpu().numpy()
                x_center, _, width, height = boxes.xywh[i].cpu().numpy()

                # bbox 넓이 계산 및 퍼블리시
                x1, y1, x2, y2 = map(int, (x1, y1, x2, y2))
                bbox_size = float((x2 - x1) * (y2 - y1))
                bbox_msg = Float32()
                bbox_msg.data = bbox_size
                self.bbox_publisher.publish(bbox_msg)

                # 조향각 계산
                if class_name == "Vest":
                    error = image_center_x - x_center
                    angle = (error / (frame_width / 2)) * max_angle
                    angle_msg = Float32()
                    angle_msg.data = angle
                    self.steering_publisher.publish(angle_msg)
                    self.get_logger().info(f"[퍼블리시] 조향각: {angle:.2f}°")

                # 텍스트 표시
                cv2.rectangle(frame, (x1, y1), (x2, y2), BOUNDING_BOX_COLOR, 2)
                label = f"{class_name} | {bbox_size:.0f}px | {angle:.1f}°" if angle else f"{class_name} | {bbox_size:.0f}px"
                cv2.putText(frame, label, (x1, max(y1 - 10, 0)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, BOUNDING_BOX_COLOR, 2)
                break  # 첫 번째 객체만 처리

        # 화면 출력
        cv2.imshow("YOLOv8 BoundingBox + Steering", frame)
        if cv2.waitKey(1) & 0xFF == 27:
            rclpy.shutdown()

    def destroy(self):
        self.cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = YoloVideoPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 사용자 종료")
    finally:
        node.destroy()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

