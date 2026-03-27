import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float64MultiArray
import math

class LocalFrameConverter(Node):
    def __init__(self):
        super().__init__('local_frame_converter')  # Needed
        self.starting_lat = None
        self.starting_lon = None
        # self.coordinates_subscription = self.create_subscription(Vector3Stamped, "/filter/postionlla", self.latlon_to_xy, 10)
        self.coordinates_subscription = self.create_subscription(Float64MultiArray, '/gps/latlon', self.latlon_to_xy, 10)
        # Earth radius in meters
        self.R = 6378137.0

    def latlon_to_xy(self, data):
        if self.starting_lat == None or self.starting_lon == None:
            self.starting_lat = data.data[0]
            self.starting_lon = data.data[1]

        current_lat = data.data[0]
        current_lon = data.data[1]

        dlat = math.radians(current_lat - self.starting_lat)
        dlon = math.radians(current_lon - self.starting_lon)

        x = dlon * math.cos(math.radians(self.starting_lat)) * self.R
        y = dlat * self.R

        self.get_logger().info(f"meters: ({x}, {y})")
        return x, y

def main(args=None):
    rclpy.init(args=args)
    node = LocalFrameConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
