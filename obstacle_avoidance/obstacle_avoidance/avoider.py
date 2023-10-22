"""ROS wrapper around ReactiveController: scan in, cmd_vel out."""
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from obstacle_avoidance.reactive import ReactiveController


class Avoider(Node):

    def __init__(self):
        super().__init__('avoider')
        d = self.declare_parameter
        self.ctrl = ReactiveController(
            cruise_speed=d('cruise_speed', 0.5).value,
            turn_speed=d('turn_speed', 1.2).value,
            stop_distance=d('stop_distance', 0.6).value,
            clear_distance=d('clear_distance', 0.9).value,
            front_half_angle=math.radians(d('front_half_angle_deg', 25.0).value),
            side_angle=math.radians(d('side_angle_deg', 80.0).value),
            steer_gain=d('steer_gain', 0.4).value,
        )
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.last_state = None

    def on_scan(self, scan):
        v, w, s = self.ctrl.compute(scan.ranges, scan.angle_min, scan.angle_increment, scan.range_max)
        if self.ctrl.state != self.last_state:
            self.get_logger().info(
                f'{self.ctrl.state:<7} front={s["front"]:.2f} left={s["left"]:.2f} right={s["right"]:.2f}')
            self.last_state = self.ctrl.state
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = Avoider()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
