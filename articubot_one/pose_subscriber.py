import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray

class PoseSubscriber(Node):
    def __init__(self):
        super().__init__("pose_subscribe_node")
        self.pose_subscriber = self.create_subscription(PoseArray, '/world/empty/pose/info', self.callback_function, 10)

    def callback_function(self, poseArray):
        element = poseArray.poses[1]
        self.get_logger().info(f"x={element.position.x}, y={element.position.y}")

def main(args=None):
    rclpy.init(args=args)
    node = PoseSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()