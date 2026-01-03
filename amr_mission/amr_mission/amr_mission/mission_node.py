import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import time

class MissionNode(Node):
    def __init__(self):
        super().__init__('mission_node')
        self.publisher_ = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.start_time = time.time()
        self.get_logger().info('Mission Node Started! Robot will move in a square.')

    def timer_callback(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        elapsed = time.time() - self.start_time
        cycle = elapsed % 8.0

        if cycle < 2.0:
            msg.twist.linear.x = 0.2
            self.get_logger().info('Forward')
        elif cycle < 4.0:
            msg.twist.angular.z = 0.5
            self.get_logger().info('Turn')
        elif cycle < 6.0:
            msg.twist.linear.x = 0.2
            self.get_logger().info('Forward')
        else:
            msg.twist.angular.z = 0.5
            self.get_logger().info('Turn')

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
