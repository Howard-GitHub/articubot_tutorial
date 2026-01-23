import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage

class OdometrySubscriber(Node):
    def __init__(self):
        super().__init__("odom_subscribe_node")
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.callback_function, 10)
        # self.odom_subscriber = self.create_subscription(TFMessage, '/tf', self.callback_function, 10)


    def callback_function(self, odom_msg):
        self.get_logger().info(f"odom data: ({round(odom_msg.pose.pose.position.x, 1)}, {round(odom_msg.pose.pose.position.y, 1)})")
        # self.get_logger().info(f"tf: {tf}")
        

def main(args=None):
    rclpy.init(args=args)
    node = OdometrySubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()