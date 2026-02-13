import math
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster



class TfCombined(Node):
    def __init__(self):
        super().__init__('tf_combined_node')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.t = 0.0    

    def timer_callback(self):
        self.t += 0.05

        # Moving frame 계산
        x_moving = 2.0 * math.cos(self.t)
        y_moving = 2.0 * math.sin(self.t)
        z_moving = 0.0
        yaw_moving = math.atan2(-y_moving, -x_moving)
        q_moving = quaternion_from_euler(0.0, 0.0, yaw_moving)

        moving_msg = TransformStamped()
        moving_msg.header.stamp = self.get_clock().now().to_msg()
        moving_msg.header.frame_id = 'world'
        moving_msg.child_frame_id = 'moving_frame'
        moving_msg.transform.translation.x = x_moving
        moving_msg.transform.translation.y = y_moving
        moving_msg.transform.translation.z = z_moving
        moving_msg.transform.rotation.x = q_moving[0]
        moving_msg.transform.rotation.y = q_moving[1]
        moving_msg.transform.rotation.z = q_moving[2]
        moving_msg.transform.rotation.w = q_moving[3]

        # Child frame 계산
        angle_child = 2.0 * self.t
        x_child = 1.0 * math.cos(angle_child)
        y_child = 1.0 * math.sin(angle_child)
        z_child = 0.0
        yaw_child = math.atan2(-y_child, -x_child)
        q_child = quaternion_from_euler(0.0, 0.0, yaw_child)

        t_child = TransformStamped()
        t_child.header.stamp = self.get_clock().now().to_msg()
        t_child.header.frame_id = 'moving_frame'
        t_child.child_frame_id = 'child_frame'
        t_child.transform.translation.x = x_child
        t_child.transform.translation.y = y_child
        t_child.transform.translation.z = z_child
        t_child.transform.rotation.x = q_child[0]
        t_child.transform.rotation.y = q_child[1]
        t_child.transform.rotation.z = q_child[2]
        t_child.transform.rotation.w = q_child[3]

        # 두 프레임 브로드캐스트
        self.tf_broadcaster.sendTransform(moving_msg)
        self.tf_broadcaster.sendTransform(t_child)

def main(args=None):
    import rclpy
    rclpy.init(args=args)
    node = TfCombined()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()