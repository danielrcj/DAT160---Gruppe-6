import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64
from geometry_msgs.msg import Pose, Point
from scoring_interfaces.srv import SetMarkerPosition
import sys

class AucroDetection(Node):
    def __init__(self, namespace):
        super().__init__('aucro_dect')

        self.namespace = namespace

        # Subscriptions
        self.create_subscription(Pose, f'/{self.namespace}/marker_map_pose', self.marker_pose_callback, 10)
        self.create_subscription(Int64, f'/{self.namespace}/marker_id', self.marker_id_callback, 10)

        # Publisher
        self.request_help_pub = self.create_publisher(Point, 'request_help_pub', 10)

        # Service client
        self.client = self.create_client(SetMarkerPosition, '/set_marker_position')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /set_marker_position service...')

        self.marker_data = {"pose": None, "id": None}
        self.reported_markers = set()

        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def marker_pose_callback(self, msg):
        self.marker_data["pose"] = msg
        self.get_logger().info(f"[{self.namespace}] Marker position updated: {msg.position}")

    def marker_id_callback(self, msg):
        self.marker_data["id"] = int(msg.data)
        self.get_logger().info(f"[{self.namespace}] Marker ID received: {msg.data}")

    def timer_callback(self):
        # Request help if marker ID is 4 (big fire)
        if self.marker_data["id"] == 4:
            self.request_help(self.marker_data["pose"])

        self.report_marker()

    def report_marker(self):
        if self.marker_data["pose"] is not None and self.marker_data["id"] is not None:
            req = SetMarkerPosition.Request()
            req.marker_id = self.marker_data["id"]
            req.marker_position = self.marker_data["pose"].position

            self.get_logger().info(f"[{self.namespace}] found marker ID {req.marker_id} at position {req.marker_position}")
            future = self.client.call_async(req)
            future.add_done_callback(self.handle_service_response)

    def request_help(self, pose):
        pos = Point()
        pos.x = pose.position.x
        pos.y = pose.position.y
        pos.z = pose.position.z

        self.request_help_pub.publish(pos)

    def handle_service_response(self, future):
        try:
            response = future.result()
            if response.accepted:
                self.get_logger().info(f"[{self.namespace}] Marker successfully reported.")
                self.reported_markers.add(self.marker_data["id"])
                self.marker_data = {"pose": None, "id": None}  # Reset after reporting
            else:
                self.get_logger().info(f"[{self.namespace}] Marker reporting failed.")
        except Exception as e:
            self.get_logger().error(f"[{self.namespace}] Service call failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    robot_name = 'tb3_0' 

    if len(sys.argv) > 1:
        robot_name = sys.argv[1]

    node = AucroDetection(robot_name)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
