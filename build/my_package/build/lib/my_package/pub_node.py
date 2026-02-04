import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class PubNode(Node):
    def __init__(self):
        super().__init__('pub_node')
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        # Declare and get the model parameter
        self.declare_parameter('model', '')  
        self.model = self.get_parameter('model').value

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 1.0
        self.pub.publish(msg)

        #self.get_logger().info(f'Publishing: "{self.model}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    pub_node = PubNode()
    rclpy.spin(pub_node)
    pub_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()