import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy

QoS_Test = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
    reliability=QoSReliabilityPolicy.RELIABLE
)

class CameraPublisher(Node):
    def __init__(self, video_device='/dev/video4'):
        super().__init__('camera_publisher')
        self.publisher_0 = self.create_publisher(Image, 'network_test_0', QoS_Test)
        # self.publisher_1 = self.create_publisher(Image, 'network_test_1', QoS_Test)
        # self.publisher_2 = self.create_publisher(Image, 'network_test_2', QoS_Test)
        # self.publisher_3 = self.create_publisher(Image, 'network_test_3', QoS_Test)
        self.video_device = video_device
        self.cap = cv2.VideoCapture(self.video_device)
        timer_period = 0.1  # 0.1 seconds (adjust as needed)
        self.timer = self.create_timer(timer_period, self.publish_image)
        self.bridge = CvBridge()

    def grayscale(self, img):
        """Applies the Grayscale transform
        This will return an image with only one color channel
        but NOTE: to see the returned image as grayscale
        you should call plt.imshow(gray, cmap='gray')"""
        return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    def convert_to_hsv(self, img):
        """Converts an image to HSV color space"""
        return cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    def publish_image(self):
        ret, frame = self.cap.read()
        if ret:
            # # Convert the OpenCV image to a grayscale image
            # gray_frame = self.grayscale(frame)
            # small_gray_frame = cv2.resize(gray_frame, (320, 240))
            # ros_image_gray = self.bridge.cv2_to_imgmsg(small_gray_frame, encoding="mono8")
            # cv2.imshow('Grayscale Image', small_gray_frame)
            
            HSV_frame = self.convert_to_hsv(frame)
            small_HSV_frame = cv2.resize(HSV_frame, (320, 240))
            ros_image = self.bridge.cv2_to_imgmsg(small_HSV_frame, encoding="rgb8")
            cv2.imshow('HSV Image', small_HSV_frame)

            # small_color_frame = cv2.resize(frame, (640, 480))
            # ros_image = self.bridge.cv2_to_imgmsg(small_color_frame, encoding="bgr8")
            # cv2.imshow('Color Image', small_color_frame)

            # Publish the ROS Image message
            self.publisher_0.publish(ros_image)
            # self.publisher_1.publish(ros_image_gray)

            # Display the original and grayscale images
            # cv2.imshow('Camera Image', frame)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    CameraPublisher_node = CameraPublisher()
    rclpy.spin(CameraPublisher_node)
    CameraPublisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
