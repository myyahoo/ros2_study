from rclpy.node import Node
from std_msgs.msg import Float32
import tf2_ros
import rclpy

class TfDistancePublisher(Node):
    def __init__(self):
        super().__init__('tf_distance_publisher')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(Float32, 'tf_distance', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        try:
            # 'world' 프레임에서 'moving_frame' 프레임으로의 변환을 조회    
            transform = self.tf_buffer.lookup_transform('world', 'child_frame', rclpy.time.Time())
            # 변환에서 x, y, z 거리 계산
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z
            distance = (x**2 + y**2 + z**2)**0.5

            self.pub.publish(Float32(data=distance))
            self.get_logger().info(f'Distance from world to moving_frame: {distance:.2f} m')
        except tf2_ros.LookupException:
            self.get_logger().warn('Transform not found: world to moving_frame')
        except tf2_ros.ExtrapolationException:
            self.get_logger().warn('Extrapolation error when looking up transform')     
        
def main(args=None):
    rclpy.init(args=args)
    node = TfDistancePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()