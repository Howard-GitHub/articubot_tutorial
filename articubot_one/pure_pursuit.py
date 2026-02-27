import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Twist
from tf_transformations import euler_from_quaternion
import math

class OrientationTracker(Node):
    def __init__(self):
        super().__init__("pure_pursuit")
        self.pose_subscriber = self.create_subscription(PoseArray, '/world/empty/pose/info', self.outputOrientation, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.linePath = [(0, 0), (1.448, 1.85)]

    def outputOrientation(self, poseArray):
        vehicleInfo = poseArray.poses[1]

        # store quaternion values
        quaternion_value = [
            vehicleInfo.orientation.x,
            vehicleInfo.orientation.y,
            vehicleInfo.orientation.z,
            vehicleInfo.orientation.w
        ]

        # convert quaternion values to degree angles
        _, _, yaw_rad = euler_from_quaternion(quaternion_value)
        yaw_deg = math.degrees(yaw_rad)
        if yaw_deg < 0:
            yaw_deg += 360

        self.calculateTurnAngle(yaw_deg, vehicleInfo)


    def calculateTurnAngle(self, vehicle_heading, vehicleInfo):
        # target position dummy values (do not have this information yet as its needed from the line intersection method)
        targetX = 1.448
        targetY = -1.85

        # current vehicle position
        currentX = vehicleInfo.position.x
        currentY = vehicleInfo.position.y

        # calculates the angle of the target relative to the current position of the vehicle
        target_angle = math.degrees(math.atan2(targetY - currentY, targetX - currentX))

        # calculates how much the vehicle can turn to move toward the target point
        turn_angle = target_angle - vehicle_heading

        # calculates which direction to turn provides the minimum turn angle
        if turn_angle > 180:
            turn_angle = turn_angle - 360
        elif turn_angle < -180:
            turn_angle = turn_angle + 360

        self.get_logger().info(f"turn angle: {turn_angle} | vehicle heading: {vehicle_heading}")

        # velocity_command = Twist() 
        # velocity_command.linear.x = 0.5
        # velocity_command.angular.z = turn_angle * (2.0 / 180.0)

        # self.cmd_vel_publisher.publish(velocity_command)


def main(args=None):
    rclpy.init(args=args)
    node = OrientationTracker()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()