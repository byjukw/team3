import cv2
from ultralytics import YOLO

# 새로운 모델 로드
model_path = r"C:\Users\bln08\OneDrive\바탕 화면\스마트운행체설계프로젝트\유니폼 인식\best.pt"
model = YOLO(model_path) 
# 비디오 로드
video_path = r"C:\Users\bln08\OneDrive\바탕 화면\스마트운행체설계프로젝트\유니폼 인식\vest_test.mp4"
cap = cv2.VideoCapture(video_path)

# 비디오가 정상적으로 열리는지 확인
if not cap.isOpened():
    print("Error: Could not open video.")
    exit()

while True:
    # 프레임 읽기
    ret, frame = cap.read()
    
    if not ret:
        print("End of video.")
        break  # 비디오 종료
    
    # 객체 감지 수행
    results = model(frame)
    
    # 감지된 객체 표시 (Vest 클래스만 출력)
    frame_with_results = frame.copy()
    
    for result in results[0].boxes:
        x_center, y_center, width, height = result.xywh[0].tolist()
        conf = result.conf[0].item()
        cls = result.cls[0].item()
        class_name = model.names[int(cls)]
        
        if class_name == "Vest":  # Vest 클래스만 출력 및 바운딩 박스 그리기
            print(f"클래스: {class_name}, 신뢰도: {conf:.2f}, 좌표: ({x_center}, {y_center}, {width}, {height})")
            x1, y1 = int(x_center - width / 2), int(y_center - height / 2)
            x2, y2 = int(x_center + width / 2), int(y_center + height / 2)
            cv2.rectangle(frame_with_results, (x1, y1), (x2, y2), (0, 255, 0), 3)  
            cv2.putText(frame_with_results, f"{class_name} ({conf:.2f})", (x1, y1 - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA) 
    
    # 감지된 이미지 화면에 표시
    cv2.imshow("Detected Video", frame_with_results)
    
    # ESC 키를 누르면 종료
    if cv2.waitKey(1) & 0xFF == 27:
        break

# 비디오 객체 해제
cap.release()
cv2.destroyAllWindows()
