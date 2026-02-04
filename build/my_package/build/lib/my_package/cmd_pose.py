from rclpy.node import Node
from my_package_msgs.msg import CmdAndPoseVel

class CmdPoseNode(Node):
    def __init__(self):
        super().__init__('cmd_pose_node')
        self.publisher_ = self.create_publisher(CmdAndPoseVel, 'cmd_and_pose_vel', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = CmdAndPoseVel()
        msg.cmd_vel_linear = float(self.i)
        msg.cmd_vel_angular = float(self.i) * 0.1
        msg.pose_x = float(self.i) * 0.5
        msg.pose_y = float(self.i) * 0.2

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg}')
        self.i += 1

def main(args=None):
    import rclpy
    rclpy.init(args=args)
    cmd_pose_node = CmdPoseNode()
    rclpy.spin(cmd_pose_node)
    cmd_pose_node.destroy_node()
    rclpy.shutdown()
    