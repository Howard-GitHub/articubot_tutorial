import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Twist
from tf_transformations import euler_from_quaternion
import math
import numpy as np


def generate_dubins_path(x, y, theta, R, xg, yg, spacing):
    """
    Generate a shortest circle → tangent → goal path (right or left turn),
    and convert all path points to the right-hand rule frame:
        (x, y) -> (y, -x)
    """

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
            nx, ny = -math.sin(theta), math.cos(theta)
            clockwise = True
        else:
            nx, ny = math.sin(theta), -math.cos(theta)
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
                heading = np.array([math.sin(phi), -math.cos(phi)])
            else:
                heading = np.array([-math.sin(phi), math.cos(phi)])

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

    # Convert to Gazebo coordinates (right-hand rule)
    # (x, y) -> (y, -x)
    converted = [(py, -px) for (px, py) in best_global]
    return converted


class OrientationTracker(Node):
    def __init__(self):
        super().__init__("pure_pursuit")

        self.pose_subscriber = self.create_subscription(
            PoseArray,
            '/world/empty/pose/info',
            self.outputOrientation,
            10
        )
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.linePath = generate_dubins_path(
            x=0.0,
            y=0.0,
            theta=math.radians(90),
            R=1.0,
            xg=-0.1,
            yg=-8.0,
            spacing=0.1
        )

        print("Generated path:")
        print(self.linePath)

        # Pure Pursuit parameters
        self.lookahead_distance = 1.5
        self.goal_tolerance = 0.35
        self.linear_speed = 0.5
        self.max_angular_z = 1.5

        # Helpful state for debugging and forward progress
        self.last_closest_index = 0

        print("Initial lookahead distance:", self.lookahead_distance)

    def outputOrientation(self, poseArray):
        vehicleInfo = poseArray.poses[1]
        current_x = vehicleInfo.position.x
        current_y = vehicleInfo.position.y

        # Stop if near final goal
        goal_x, goal_y = self.linePath[-1]
        dist_to_goal = math.hypot(goal_x - current_x, goal_y - current_y)

        if dist_to_goal < self.goal_tolerance:
            velocity_command = Twist()
            velocity_command.linear.x = 0.0
            velocity_command.angular.z = 0.0
            self.cmd_vel_publisher.publish(velocity_command)
            # print("Reached goal.")
            return

        lookahead_point, closest_index = self.find_lookahead_point(
            current_x,
            current_y,
            self.lookahead_distance
        )

        self.last_closest_index = closest_index

        # Get yaw from quaternion
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

        print(
            f"Vehicle: ({current_x:.2f}, {current_y:.2f}) | "
            f"Closest idx: {closest_index} | "
            f"Lookahead: ({lookahead_point[0]:.2f}, {lookahead_point[1]:.2f}) | "
            f"Yaw: {yaw_deg:.2f}"
        )

        self.calculateTurnAngle(
            yaw_deg,
            vehicleInfo,
            lookahead_point[0],
            lookahead_point[1]
        )

    def find_lookahead_point(self, current_x, current_y, lookahead_distance):
        """
        Find the closest path point to the vehicle, then move forward along
        the path by 'lookahead_distance' and return that point.
        """

        # Only search forward from the last closest index so the controller
        # doesn't latch onto old parts of the path behind the vehicle.
        min_dist = float("inf")
        closest_index = self.last_closest_index

        for i in range(self.last_closest_index, len(self.linePath)):
            px, py = self.linePath[i]
            dist = math.hypot(px - current_x, py - current_y)
            if dist < min_dist:
                min_dist = dist
                closest_index = i

        # Walk forward along the path by lookahead_distance
        accumulated = 0.0

        for i in range(closest_index, len(self.linePath) - 1):
            x1, y1 = self.linePath[i]
            x2, y2 = self.linePath[i + 1]
            seg_len = math.hypot(x2 - x1, y2 - y1)

            if accumulated + seg_len >= lookahead_distance:
                remain = lookahead_distance - accumulated
                ratio = remain / seg_len if seg_len > 1e-9 else 0.0

                look_x = x1 + ratio * (x2 - x1)
                look_y = y1 + ratio * (y2 - y1)
                return (look_x, look_y), closest_index

            accumulated += seg_len

        # If we run out of path, return the goal
        return self.linePath[-1], closest_index

    def calculateTurnAngle(self, vehicle_heading, vehicleInfo, targetX, targetY):
        currentX = vehicleInfo.position.x
        currentY = vehicleInfo.position.y

        # Angle from vehicle to lookahead point
        target_angle = math.degrees(math.atan2(targetY - currentY, targetX - currentX))

        # Heading error
        turn_angle = target_angle - vehicle_heading

        # Wrap to [-180, 180]
        if turn_angle > 180:
            turn_angle -= 360
        elif turn_angle < -180:
            turn_angle += 360

        velocity_command = Twist()
        velocity_command.linear.x = self.linear_speed
        velocity_command.angular.z = turn_angle * (2.0 / 180.0)

        # Clamp angular velocity a bit for stability
        if velocity_command.angular.z > self.max_angular_z:
            velocity_command.angular.z = self.max_angular_z
        elif velocity_command.angular.z < -self.max_angular_z:
            velocity_command.angular.z = -self.max_angular_z

        self.cmd_vel_publisher.publish(velocity_command)


def main(args=None):
    rclpy.init(args=args)
    node = OrientationTracker()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()