import rclpy
from rclpy.node import Node
from rclpy.timer import Timer
from std_msgs.msg import Bool
from ultralytics import YOLO
import os

class YOLOInferenceNode(Node):
    def __init__(self):
        super().__init__('yolo_inference_node')

        self.declare_parameter('model_path', '/home/rail/project/best_box.pt')
        self.declare_parameter('image_path', '/home/rail/project/frame.jpg')

        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.image_path = self.get_parameter('image_path').get_parameter_value().string_value

        if not os.path.exists(self.model_path):
            self.get_logger().error(f"모델 파일 없음: {self.model_path}")
            return
        self.model = YOLO(self.model_path)
        self.get_logger().info(f"YOLO 모델 로드 완료: {self.model_path}")

        self.publisher_ = self.create_publisher(Bool, '/yolo/class0_detected', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)  # 1초마다 실행

    def timer_callback(self):
        detected = False

        if os.path.exists(self.image_path):
            results = self.model(self.image_path)[0]
            class_ids = results.boxes.cls.tolist() if results.boxes is not None else []
            detected = 0 in class_ids

        msg = Bool()
        msg.data = detected
        self.publisher_.publish(msg)
        self.get_logger().info(f"클래스 0 {'감지됨' if detected else '감지되지 않음'}")

def main(args=None):
    rclpy.init(args=args)
    node = YOLOInferenceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()