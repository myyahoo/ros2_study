from my_package_msgs.srv import MultiSpawn
from rclpy.node import Node 
from turtlesim.srv import TeleportAbsolute
import rclpy

class MyServiceNode(Node):
    def __init__(self):
        super().__init__('my_service_node')
        self.srv = self.create_service(MultiSpawn,'multi_spawn', self.handle_multi_spawn)
        self.client = self.create_client(TeleportAbsolute, 'turtle1/teleport_absolute')


    def handle_multi_spawn(self, request, response):
        self.get_logger().info(f'Received request: num={request.num}')
        response.x = [1.,2.,3.]
        response.y = [4.,5.,6.]
        self.get_logger().info(f'Sending response: x={response.x}, y={response.y}')
        self.client.call_async(TeleportAbsolute.Request(x=2.0, y=3.0, theta=0.0))
        return response
    

def main(args=None):
    rclpy.init(args=args)
    my_service_node = MyServiceNode()
    rclpy.spin(my_service_node)
    my_service_node.destroy_node()
    rclpy.shutdown()    

if __name__ == '__main__':
    main()

