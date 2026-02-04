import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription_ = self.create_subscription(
            Image,
            '/image_raw',
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        
    def  listener_callback(self, msg):
        try:

            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.rectangle(cv_image, (50, 50), (200, 200), (0, 255, 0), 2)   
            cv2.imshow('Received Image', cv_image)
            cv2.waitKey(1)
            self.get_logger().info('Image received and displayed')
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}') 

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()        

if __name__ == '__main__':
    main()
