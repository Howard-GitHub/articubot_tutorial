import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Twist
from tf_transformations import euler_from_quaternion
import math

class SetVehicleHeading(Node):
    def __init__(self):
        super().__init__("set_vehicle_pose")
        # self.target_coordinates = self.coordinates_input()
        self.angle_orientation = self.orientation_input()
        self.pose_subscriber = self.create_subscription(PoseArray, '/world/empty/pose/info', self.turn_to_target, 10)
        self.pose_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    def orientation_input(self):
        angle = float(input("Enter angle: "))
        return angle
    
    def turn_to_target(self, pose_array):
        vehicle_pose = pose_array.poses[1]

        # store quaternion values
        quaternion_value = [
            vehicle_pose.orientation.x,
            vehicle_pose.orientation.y,
            vehicle_pose.orientation.z,
            vehicle_pose.orientation.w
        ]

        # convert quaternion values to degree angles
        _, _, yaw_rad = euler_from_quaternion(quaternion_value)
        vehicle_heading = math.degrees(yaw_rad)
        remaining_turn_angle = self.angle_orientation - vehicle_heading
    
        if remaining_turn_angle < 0:
            angular_velocity = -0.3
        else:
            angular_velocity = 0.3

        self.get_logger().info(str(remaining_turn_angle))

        if ((179.85 <= self.angle_orientation <= 180) or (-180 <= self.angle_orientation <= -179.0)) and ((0 <= remaining_turn_angle < 0.15) or (-360 < remaining_turn_angle < -359.0)):
            print("Reached target heading")
            velocity_cmd = Twist()
            velocity_cmd.angular.z = 0.0
            self.pose_publisher.publish(velocity_cmd)
            self.destroy_node()
        elif (remaining_turn_angle < 0.15) and (remaining_turn_angle > -0.15):
            print("Reached target heading")
            velocity_cmd = Twist()
            velocity_cmd.angular.z = 0.0
            self.pose_publisher.publish(velocity_cmd)
            self.destroy_node()
        else:
            velocity_cmd = Twist()
            velocity_cmd.angular.z = angular_velocity
            self.pose_publisher.publish(velocity_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = SetVehicleHeading()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()



