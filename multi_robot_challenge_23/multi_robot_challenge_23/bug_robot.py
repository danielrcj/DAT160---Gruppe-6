#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import sys

class BugRobot(Node):
    def __init__(self, robot_name, follow_side='right'):
        super().__init__('bug_robot')
        self.robot_name = robot_name
        self.follow_side = follow_side.lower()

        # Subscribers and publishers with proper namespacing
        self.laser_subscriber = self.create_subscription(
            LaserScan,
            f'/{robot_name}/scan',
            self.laser_callback,
            10
        )
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            f'/{robot_name}/cmd_vel',
            10
        )

        # Wall-following variables
        self.active = True
        self.regions = None
        self.state = 0
        self.state_dict = {
            0: 'find the wall',
            1: 'turn',
            2: 'follow the wall',
        }

        # Timers
        self.create_timer(0.1, self.wall_follower)

    def laser_callback(self, msg):
        # Process laser scan data for wall following
        ranges = msg.ranges
        if self.follow_side == 'right':
            self.regions = {
                'far': min(min(ranges[0:9] + ranges[350:359]), 10.0),
                'front': min(min(ranges[0:9] + ranges[350:359]), 10.0),
                'f_side': min(min(ranges[300:339]), 10.0),
                'side': min(min(ranges[270:309]), 10.0),
            }
        elif self.follow_side == 'left':
            self.regions = {
                'far': min(min(ranges[0:9] + ranges[350:359]), 10.0),
                'front': min(min(ranges[0:9] + ranges[350:359]), 10.0),
                'f_side': min(min(ranges[20:59]), 10.0),
                'side': min(min(ranges[50:89]), 10.0),
            }

    def change_state(self, state):
        if state != self.state:
            self.get_logger().info(f'Wall follower - [{state}] - {self.state_dict[state]}')
            self.state = state

    def take_action(self):
        regions = self.regions

        d = 1.0  # Distance threshold

        if regions['front'] > d and regions['f_side'] > d and regions['side'] > d:
            self.change_state(0)
        elif regions['front'] < d:
            self.change_state(1)
        elif regions['f_side'] < d:
            self.change_state(2)
        else:
            self.change_state(0)

    def find_wall(self):
        msg = Twist()
        msg.linear.x = 0.3
        if self.follow_side == 'right':
            msg.angular.z = -0.3  # Turn right
        else:
            msg.angular.z = 0.3   # Turn left
        return msg

    def turn(self):
        msg = Twist()
        if self.follow_side == 'right':
            msg.angular.z = 0.5  # Turn left to avoid obstacle
        else:
            msg.angular.z = -0.5  # Turn right to avoid obstacle
        return msg

    def follow_the_wall(self):
        msg = Twist()
        msg.linear.x = 0.5
        return msg

    def wall_follower(self):
        if not self.active or self.regions is None:
            return

        self.take_action()

        if self.state == 0:
            cmd = self.find_wall()
        elif self.state == 1:
            cmd = self.turn()
        elif self.state == 2:
            cmd = self.follow_the_wall()
        else:
            self.get_logger().error('Unknown state!')
            cmd = Twist()

        # Publish velocity command
        self.cmd_vel_publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)

    # Parse arguments
    robot_name = 'tb3_0'
    follow_side = 'right'

    if len(sys.argv) > 1:
        robot_name = sys.argv[1]
    if len(sys.argv) > 2:
        follow_side = sys.argv[2]

    node = BugRobot(robot_name, follow_side)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
