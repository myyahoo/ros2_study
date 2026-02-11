import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MyPubNode(Node):
    def __init__(self):
        super().__init__('my_pub_node')
        self.declare_parameter('arg1', 'default_value')

        self.arg1 = self.get_parameter('arg1').value
        self.get_logger().info(f'Parameter arg1: {self.arg1}')
        self.pub = self.create_publisher(Twist,'/cmd_vel',10)
                                              
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.0
        self.pub.publish(msg)
        self.get_logger().info('Published empty Twist message') 


def main(args=None):
    rclpy.init(args=args)
    node = MyPubNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()