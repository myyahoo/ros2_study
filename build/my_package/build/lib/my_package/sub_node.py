import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from yolo_msgs.msg import Detection, DetectionArray, Point2D, BoundingBox2D


class SubNode(Node):
    def __init__(self):
        super().__init__('sub_node')
        self.sub = self.create_subscription(
            DetectionArray,
            'detections',
            self.listener_callback,
            10)
         # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Received linear.x: "{msg}"')
    

def main(args=None):
    rclpy.init(args=args)
    sub_node = SubNode()
    rclpy.spin(sub_node)
    sub_node.destroy_node()
    rclpy.shutdown()    

if __name__ == '__main__':
    main()