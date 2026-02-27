import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Twist
from tf_transformations import euler_from_quaternion
import math

# Logic:
# 1. Take x and y input
# 2. Take orientation in radians
# 3. Modify Gazebo's axis to reflect traditional x and y axis
# 4. Move vehicle too coordinate by sending command velocities
#     - rotate vehicle to face target
#     - move vehicle to the target
#     - orient vehicle to face the desired direction


class SetVehicleHeading(Node):
    def __init__(self):
        super().__init__("set_vehicle_pose")
        # self.target_coordinates = self.coordinates_input()
        self.angle_orientation = self.orientation_input()
        self.pose_subscriber = self.create_subscription(PoseArray, '/world/empty/pose/info', self.turn_to_target, 10)
        self.pose_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    # def coordinates_input(self):
    #     input_x = float(input("Enter x: "))
    #     input_y = float(input("Enter y: "))

    #     gazebo_x = input_y
    #     gazebo_y = -1 * input_x
    #     gazebo_coordinates = [gazebo_x, gazebo_y]
        
    #     return gazebo_coordinates
    
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
        vehicle_heading = math.degrees(yaw_rad) + 180
        # if yaw_deg < 0:
        #     yaw_deg += 360
    
        # current_x = vehicle_pose.position.x
        # current_y = vehicle_pose.position.y

        # target_x = self.target_coordinates[0]
        # target_y = self.target_coordinates[1]

        # target_angle = math.degrees(math.atan2(target_y - current_y, target_x - current_x)) + 180
        remaining_turn_angle = self.angle_orientation - vehicle_heading

        angular_direction = 0
    
        if remaining_turn_angle < 0:
            angular_velocity = -0.3
        else:
            angular_velocity = 0.3

        self.get_logger().info(str(remaining_turn_angle))

        if (remaining_turn_angle < 0.15 and remaining_turn_angle > -0.15):
            velocity_cmd = Twist()
            velocity_cmd.angular.z = 0.0
            self.pose_publisher.publish(velocity_cmd)
        else:
            print("Reached target heading")
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



