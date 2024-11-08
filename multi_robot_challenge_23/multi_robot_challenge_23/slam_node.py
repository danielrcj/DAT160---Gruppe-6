import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Pose
import numpy as np

class SimpleSLAM(Node):
    def __init__(self):
        super().__init__('simple_slam')

        # Declare parameters for robot names
        self.declare_parameter('robot_names', ['robot1', 'robot2'])
        robot_names = self.get_parameter('robot_names').get_parameter_value().string_array_value

        # Subscribers for each robot's laser scan and odometry
        self.laser_subscribers = {}
        self.odom_subscribers = {}
        self.robot_states = {}

        for robot_name in robot_names:
            # Initialize state storage
            self.robot_states[robot_name] = {'pose': None, 'scan': None}

            # Subscribe to laser scans
            laser_topic = f'/{robot_name}/scan'
            self.laser_subscribers[robot_name] = self.create_subscription(
                LaserScan,
                laser_topic,
                self.create_laser_callback(robot_name),
                10
            )

            # Subscribe to odometry
            odom_topic = f'/{robot_name}/odom'
            self.odom_subscribers[robot_name] = self.create_subscription(
                Odometry,
                odom_topic,
                self.create_odom_callback(robot_name),
                10
            )

        # Publisher for the combined map
        self.map_publisher = self.create_publisher(OccupancyGrid, '/map', 10)

        # Timer to update the map periodically
        self.timer = self.create_timer(0.1, self.update_map)

        # Initialize the map (simple 2D numpy array for illustration)
        self.map_size = 100  # 100x100 grid
        self.map_resolution = 0.1  # Each cell represents 0.1 meters
        self.map = np.zeros((self.map_size, self.map_size), dtype=np.int8)

    def create_laser_callback(self, robot_name):
        def laser_callback(msg):
            self.robot_states[robot_name]['scan'] = msg
        return laser_callback

    def create_odom_callback(self, robot_name):
        def odom_callback(msg):
            self.robot_states[robot_name]['pose'] = msg.pose.pose
        return odom_callback

    def update_map(self):
        # Simplified SLAM logic
        for robot_name, state in self.robot_states.items():
            if state['pose'] is not None and state['scan'] is not None:
                self.process_scan(robot_name, state['pose'], state['scan'])

        # Publish the updated map
        occupancy_grid = self.convert_map_to_occupancy_grid()
        self.map_publisher.publish(occupancy_grid)

    def process_scan(self, robot_name, pose, scan):
        # Placeholder for actual SLAM processing
        # Update self.map based on the robot's pose and laser scan
        pass  # Implement SLAM algorithm here

    def convert_map_to_occupancy_grid(self):
        occupancy_grid = OccupancyGrid()
        occupancy_grid.header.stamp = self.get_clock().now().to_msg()
        occupancy_grid.header.frame_id = 'map'

        occupancy_grid.info.resolution = self.map_resolution
        occupancy_grid.info.width = self.map_size
        occupancy_grid.info.height = self.map_size
        occupancy_grid.info.origin.position.x = - (self.map_size * self.map_resolution) / 2
        occupancy_grid.info.origin.position.y = - (self.map_size * self.map_resolution) / 2
        occupancy_grid.info.origin.orientation.w = 1.0

        # Flatten the 2D map array and assign it to data
        occupancy_grid.data = self.map.flatten().tolist()

        return occupancy_grid

def main(args=None):
    rclpy.init(args=args)
    node = SimpleSLAM()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
