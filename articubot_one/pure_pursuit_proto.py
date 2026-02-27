import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Twist
from tf_transformations import euler_from_quaternion
import math
import numpy as np

def sgn(num):
    if num >= 0:
        return 1
    else:
        return -1

class PurePursuitPrototype(Node):
    def __init__(self):
        super().__init__("pure_pursuit_proto")
        self.pose_subscriber = self.create_subscription(PoseArray, '/world/empty/pose/info', self.pure_pursuit_prototype, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.line_path = [(0.0, 0.0), (1.0, -1.0), (0.0, -2.0), (1.0, -3.0), (0.0, -4.0)]
        self.index = 0


    def pure_pursuit_prototype(self, poseArray):
        # Retrieve vehicle coordinates
        vehicleInfo = poseArray.poses[1]
        current_position = [vehicleInfo.position.x, vehicleInfo.position.y]
        previous_target = [0, 0]
        target = [self.line_path[self.index][0], self.line_path[self.index][1]]
        
        if abs(target[0] - current_position[0]) <= 0.25 and abs(target[1] - current_position[1]) <= 0.25 and self.index < 4:
            self.index += 1
            previous_target = [self.line_path[self.index - 1][0], self.line_path[self.index - 1][1]]
            target = [self.line_path[self.index][0], self.line_path[self.index][1]]
            self.get_logger().info(f"index: {self.index}")

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

        self.line_circle_intersection(vehicleInfo.position, yaw_deg, previous_target, target, 0.8)

    def line_circle_intersection(self, currentPos, vehicle_heading, pt1, pt2, lookAheadDis):
        currentX = currentPos.x
        currentY = currentPos.y
        x1 = pt1[0]
        y1 = pt1[1]
        x2 = pt2[0]
        y2 = pt2[1]

        intersectFound = False

        x1_offset = x1 - currentX
        y1_offset = y1 - currentY
        x2_offset = x2 - currentX
        y2_offset = y2 - currentY

        dx = x2_offset - x1_offset
        dy = y2_offset - y1_offset
        dr = math.sqrt(dx**2 + dy**2)
        D = np.linalg.det([[x1_offset, x2_offset], [y1_offset, y2_offset]])
        discriminant = (lookAheadDis**2) * (dr**2) - D**2

        if discriminant >= 0:
            sol1_x = (D * dy + sgn(dy) * dx * math.sqrt(discriminant))/dr**2
            sol1_y = (-D * dx + abs(dy) * math.sqrt(discriminant))/dr**2
            sol2_x = (D * dy - sgn(dy) * dx * math.sqrt(discriminant))/dr**2
            sol2_y = (-D * dx - abs(dy) * math.sqrt(discriminant))/dr**2
            
            sol1 = [sol1_x + currentX, sol1_y + currentY]
            sol2 = [sol2_x + currentX, sol2_y + currentY]

            minX = min(x1, x2)
            maxX = max(x1, x2)
            minY = min(y1, y2)
            maxY = max(y1, y2)

            # if (minX <= sol1[0] <= maxX and minY <= sol1[1] <= maxY) or (minX <= sol2[0] <= maxX and minY <= sol2[1] <= maxY):
            #     intersectFound = True
            
            # if (minX <= sol1[0] <= maxX and minY <= sol1[1] <= maxY):
            #     self.get_logger().info(f'solution 1 is valid! {sol1}')

            # if (minX <= sol2[0] <= maxX and minY <= sol2[1] <= maxY):
            #     self.get_logger().info(f'solution 2 is valid! {sol2}')

            if (sol1[1] < sol2[1]):
                self.calculateTurnAngle(vehicle_heading, currentPos, sol1)
            else:
                self.calculateTurnAngle(vehicle_heading, currentPos, sol2)
            
            # self.get_logger().info(f'solution 1 {sol1}')
            # self.get_logger().info(f'solution 2 {sol2}')



    def calculateTurnAngle(self, vehicle_heading, vehicleInfo, targetPoint):
        # target position dummy values (do not have this information yet as its needed from the line intersection method)
        targetX = targetPoint[0]
        targetY = targetPoint[1]
        # self.get_logger().info(f"target point {targetPoint}")

        # current vehicle position
        currentX = vehicleInfo.x
        currentY = vehicleInfo.y

        # calculates the angle of the target relative to the current position of the vehicle
        target_angle = math.degrees(math.atan2(targetY - currentY, targetX - currentX))

        # calculates how much the vehicle can turn to move toward the target point
        turn_angle = target_angle - vehicle_heading

        # calculates which direction to turn provides the minimum turn angle
        if turn_angle > 180:
            turn_angle = turn_angle - 360
        elif turn_angle < -180:
            turn_angle = turn_angle + 360

        if self.index < 4:
            velocity_command = Twist() 
            velocity_command.linear.x = 0.5
            velocity_command.angular.z = turn_angle * (5.0 / 180.0)
            self.cmd_vel_publisher.publish(velocity_command)
        else:
            velocity_command = Twist() 
            velocity_command.linear.x = 0.0
            velocity_command.angular.z = 0.0
            self.cmd_vel_publisher.publish(velocity_command)
            self.get_logger().info("Destination reached!")
            



def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitPrototype()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()