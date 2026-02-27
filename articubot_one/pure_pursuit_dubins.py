import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Twist
from tf_transformations import euler_from_quaternion
import math

import math
import numpy as np

def generate_dubins_path(x, y, theta, R, xg, yg, spacing): 
    """
    Generate a shortest circle → tangent → goal path (right or left turn),
    and convert all path points to the right-hand rule frame:
        (x, y) -> (y, -x)
    """

    import math
    import numpy as np

    def wrap2pi(a):
        return (a + 2 * math.pi) % (2 * math.pi)

    def resample_path_uniform(points, ds):
        pts = np.array(points)
        seg_lens = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        s = np.insert(np.cumsum(seg_lens), 0, 0.0)
        s_new = np.arange(0.0, s[-1] + 1e-9, ds)
        x_new = np.interp(s_new, s, pts[:, 0])
        y_new = np.interp(s_new, s, pts[:, 1])
        return list(zip(x_new, y_new))

    def dense_arc(Cx, Cy, R, phi0, phi1, clockwise=True, n=800):
        if clockwise:
            if phi1 > phi0:
                phi1 -= 2 * math.pi
            phis = np.linspace(phi0, phi1, n)
        else:
            if phi1 < phi0:
                phi1 += 2 * math.pi
            phis = np.linspace(phi0, phi1, n)
        return [(Cx + R * math.cos(p), Cy + R * math.sin(p)) for p in phis]

    def dense_line(p0, p1, n=800):
        return list(zip(
            np.linspace(p0[0], p1[0], n),
            np.linspace(p0[1], p1[1], n)
        ))

    best_global = None
    best_len_global = float("inf")

    for turn in ["R", "L"]:

        if turn == "R":
            nx, ny = -math.sin(theta),  math.cos(theta)
            clockwise = True
        else:
            nx, ny =  math.sin(theta), -math.cos(theta)
            clockwise = False

        Cx = x - R * nx
        Cy = y - R * ny

        phi0 = math.atan2(y - Cy, x - Cx)

        dx = xg - Cx
        dy = yg - Cy
        D = math.hypot(dx, dy)
        if D <= R:
            continue

        alpha = math.atan2(dy, dx)
        beta = math.acos(R / D)
        candidates = [alpha + beta, alpha - beta]

        best_local = None
        best_len_local = float("inf")

        for phi in candidates:

            if clockwise:
                delta = wrap2pi(phi0 - phi)
            else:
                delta = wrap2pi(phi - phi0)

            if delta <= 1e-6:
                continue

            tx = Cx + R * math.cos(phi)
            ty = Cy + R * math.sin(phi)

            if clockwise:
                heading = np.array([ math.sin(phi), -math.cos(phi) ])
            else:
                heading = np.array([-math.sin(phi),  math.cos(phi) ])

            straight = np.array([xg - tx, yg - ty])
            if np.dot(heading, straight) <= 0:
                continue

            total_len = R * delta + np.linalg.norm(straight)

            if total_len < best_len_local:
                best_len_local = total_len
                best_local = (phi, tx, ty)

        if best_local is None:
            continue

        phi_tan, tx, ty = best_local

        arc_dense = dense_arc(Cx, Cy, R, phi0, phi_tan, clockwise)
        line_dense = dense_line((tx, ty), (xg, yg))
        dense_path = arc_dense + line_dense[1:]
        path_uniform = resample_path_uniform(dense_path, spacing)
        path_uniform.append((xg, yg))

        if best_len_local < best_len_global:
            best_len_global = best_len_local
            best_global = path_uniform

    if best_global is None:
        raise ValueError("No valid forward tangent found")

    # -------------------------
    # Convert to Gazebo coordinates (aka right hand rule)
    # (x, y) -> (y, -x)
    # -------------------------
    converted = [(py, -px) for (px, py) in best_global]
    return converted



class OrientationTracker(Node):
    def __init__(self):
        super().__init__("pure_pursuit")
        self.pose_subscriber = self.create_subscription(PoseArray, '/world/empty/pose/info', self.outputOrientation, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.linePath = generate_dubins_path(x=1.0, y=0.0, theta=math.radians(90),R=1.0, xg=-0.1, yg=-9.0, spacing=0.5)
        print(self.linePath)
        self.index = 1
        print("Current point:" + str(self.linePath[1]))

    def outputOrientation(self, poseArray):
        currentLookPoint = self.linePath[self.index]

        vehicleInfo = poseArray.poses[1]

        if (abs(currentLookPoint[0] - vehicleInfo.position.x) < 0.25) and (abs(currentLookPoint[1] - vehicleInfo.position.y) < 0.25):
            if (self.index == len(self.linePath) - 1):
                velocity_command = Twist() 
                velocity_command.linear.x = 0.0
                velocity_command.angular.z = 0.0
                self.cmd_vel_publisher.publish(velocity_command)
                return
            else:
                self.index += 1
                print("Current point:" + str(currentLookPoint))

        currentLookPoint = self.linePath[self.index]

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

        self.calculateTurnAngle(yaw_deg, vehicleInfo, currentLookPoint[0], currentLookPoint[1])


    def calculateTurnAngle(self, vehicle_heading, vehicleInfo, targetX, targetY):
        # target position dummy values (do not have this information yet as its needed from the line intersection method)
        # targetX = 9
        # targetY = -9

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

        # self.get_logger().info(f"turn angle: {turn_angle} | vehicle heading: {vehicle_heading}")

        velocity_command = Twist() 
        velocity_command.linear.x = 0.5
        velocity_command.angular.z = turn_angle * (2.0 / 180.0)

        self.cmd_vel_publisher.publish(velocity_command)


def main(args=None):
    rclpy.init(args=args)
    node = OrientationTracker()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()