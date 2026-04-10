import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import random
import math


class GPSSimulator(Node):
    def __init__(self):
        super().__init__('gpssimulator')

        # Publisher: [latitude, longitude]
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/gps/latlon',
            10
        )

        # Initial coordinates (example: Cal Poly Pomona area)
        self.lat = 34.056
        self.lon = -117.821

        # Timer: every 3 seconds
        self.timer = self.create_timer(3.0, self.update_position)

        self.get_logger().info("GPS Simulator Started")

    def update_position(self):
        # Step size in meters
        step_meters = 5.0

        # Random direction
        direction = random.choice(['north', 'south', 'east', 'west'])

        # Convert meters to degrees
        delta_lat = step_meters / 111111.0
        delta_lon = step_meters / (111111.0 * math.cos(math.radians(self.lat)))

        if direction == 'north':
            self.lat += delta_lat
        elif direction == 'south':
            self.lat -= delta_lat
        elif direction == 'east':
            self.lon += delta_lon
        elif direction == 'west':
            self.lon -= delta_lon

        # Publish message
        msg = Float64MultiArray()
        msg.data = [self.lat, self.lon]

        self.publisher.publish(msg)

        self.get_logger().info(
            f"Moved {direction.upper()} -> Lat: {self.lat:.6f}, Lon: {self.lon:.6f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = GPSSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()