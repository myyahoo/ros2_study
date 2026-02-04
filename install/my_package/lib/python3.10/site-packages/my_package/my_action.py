from rclpy.node import Node
from my_package_msgs.action import DistTurtle
from rclpy.action import ActionServer, ActionClient 
import rclpy
import time
from geometry_msgs.msg import Twist


class MyActionNode(Node):
    def __init__(self):
        super().__init__('my_action_node')
        self._action_server = ActionServer(
            self,
            DistTurtle,
            'dist_turtle',
            self.execute_callback)
        self._action_client = ActionClient(self, DistTurtle, 'dist_turtle')
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.declare_parameter('quality', 10)


    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        feedback_msg = DistTurtle.Feedback()
        total_distance = 0.0
        msg = Twist()   

        for i in range(1, 6):
            time.sleep(1)  # Simulate work being done
            total_distance += 1.0
            feedback_msg.remained_dist = 5.0 - total_distance
            goal_handle.publish_feedback(feedback_msg)  
            self.get_logger().info(f'Feedback: remained_dist={feedback_msg.remained_dist}')
            msg.linear.x = 1.0
            msg.angular.z = 0.5
            self.pub.publish(msg)  # Publish the Twist message with updated values
            self.get_logger().info(f'Publishing Twist: linear.x={msg.linear.x}, angular.z={msg.angular.z}')


        goal_handle.succeed()
        result = DistTurtle.Result()
        result.result_dist = total_distance
        self.get_logger().info(f'Goal succeeded: total_distance={total_distance}')
        return result
    
def main(args=None):
    rclpy.init(args=args)
    my_action_node = MyActionNode()
    rclpy.spin(my_action_node)
    my_action_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


