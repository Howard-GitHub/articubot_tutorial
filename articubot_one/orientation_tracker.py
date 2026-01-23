import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from tf_transformations import euler_from_quaternion
import math

class OrientationTracker(Node):
    def __init__(self):
        super().__init__("orientation_tracker")
        self.pose_subscriber = self.create_subscription(PoseArray, '/world/empty/pose/info', self.outputOrientation, 10)

    def outputOrientation(self, poseArray):
        vehicleInfo = poseArray.poses[1]

        quaternion_value = [
            vehicleInfo.orientation.x,
            vehicleInfo.orientation.y,
            vehicleInfo.orientation.z,
            vehicleInfo.orientation.w
        ]

        _, _, yaw_rad = euler_from_quaternion(quaternion_value)
        yaw_deg = math.degrees(yaw_rad)
        if yaw_deg < 0:
            yaw_deg += 360
        self.get_logger().info(f"yaw={yaw_deg}")

        self.calculateTurnAngle(yaw_deg, vehicleInfo)

    def calculateTurnAngle(self, yaw_deg, vehicleInfo):
        targetX = 0
        targetY = 0
        currentX = vehicleInfo.position.x
        currentY = vehicleInfo.position.y
        target_angle = math.degrees(math.atan2(targetY - currentY, targetX - currentX))

        turn_angle = target_angle - yaw_deg

        if turn_angle > 180:
            turn_angle = (turn_angle - 180) * -1
        elif turn_angle < -180:
            turn_angle = (turn_angle + 180) * -1


def main(args=None):
    rclpy.init(args=args)
    node = OrientationTracker()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()