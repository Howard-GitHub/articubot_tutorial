import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MyNode(Node):

    def __init__(self):
        super().__init__("first_node")
        self.get_logger().info("Hello from ROS2")
        self.twist_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        # self.twist_subscriber = self.create_subscription(Twist, '/cmd_vel', self.callback_function, 10)
        self.run_twist()
        

    # def callback_function(self, twist_msg):
    #     self.get_logger().info(f"the data is: {twist_msg}")

    def run_twist(self):
        twist_msg = Twist()
        twist_msg.linear.x = 3.0
        twist_msg.angular.z = 0.0
        self.twist_publisher.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()