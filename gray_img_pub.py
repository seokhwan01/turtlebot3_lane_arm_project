import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2

class LogitechCameraPublisher(Node):
    def __init__(self):
        super().__init__('logitech_camera_publisher')

        # ✅ 퍼블리셔: CompressedImage 메시지
        self.publisher = self.create_publisher(CompressedImage, '/camera/image/compressed', 1)
        self.bridge = CvBridge()

        # ✅ USB 카메라 열기
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("카메라를 열 수 없습니다.")
            exit()

        # ✅ 해상도 및 FPS 설정
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        # ✅ 설정 확인 로그
        width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.get_logger().info(f"카메라 설정: {width}x{height} @ {fps}fps")

        # ✅ 30fps 주기로 콜백 실행
        self.timer = self.create_timer(1 / 30.0, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("프레임 읽기 실패")
            return

        try:
            # ✅ Grayscale → 다시 BGR (압축 전용)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray_bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

            # ✅ 압축 메시지 생성
            msg = self.bridge.cv2_to_compressed_imgmsg(gray_bgr, dst_format='jpg')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_frame"

            self.publisher.publish(msg)
        except Exception as e:
            self.get_logger().warn(f"cvBridge 오류: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = LogitechCameraPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

