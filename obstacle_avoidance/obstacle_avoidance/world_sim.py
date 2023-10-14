"""2D simulator node: unicycle robot + simulated 2D LiDAR in a World.

Subscribes: cmd_vel
Publishes:  scan (sensor_msgs/LaserScan), odom (nav_msgs/Odometry),
            obstacles (visualization_msgs/MarkerArray), robot_marker (Marker)
TF:         odom -> base_link (dynamic), base_link -> laser (static)
"""
import math
import random

import rclpy
from geometry_msgs.msg import Point, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from obstacle_avoidance.world import World


def yaw_quat(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class WorldSim(Node):

    def __init__(self):
        super().__init__('world_sim')
        d = self.declare_parameter
        width = d('world_width', 10.0).value
        height = d('world_height', 10.0).value
        circles = d('obstacles', [3.0, 3.0, 0.5, 7.0, 7.0, 0.7, 6.5, 2.5, 0.4, 2.5, 7.5, 0.6]).value
        walls = d('walls', [4.5, 5.0, 8.0, 5.0]).value
        self.world = World.from_flat(width, height, list(circles), list(walls))

        self.x = d('start_x', 1.0).value
        self.y = d('start_y', 1.0).value
        self.theta = d('start_theta', 0.8).value
        self.robot_radius = d('robot_radius', 0.2).value
        self.beams = d('scan_beams', 180).value
        self.range_min = d('range_min', 0.12).value
        self.range_max = d('range_max', 6.0).value
        self.noise = d('range_noise_std', 0.01).value
        self.laser_offset = d('laser_x_offset', 0.1).value
        rate = d('rate', 50.0).value
        scan_rate = d('scan_rate', 10.0).value

        self.cmd = Twist()
        self.last_cmd = self.get_clock().now()
        self.collisions = 0

        self.create_subscription(Twist, 'cmd_vel', self.on_cmd, 10)
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'obstacles', 10)
        self.robot_pub = self.create_publisher(Marker, 'robot_marker', 10)
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        self.publish_static_laser()

        self.dt = 1.0 / rate
        self.create_timer(self.dt, self.step)
        self.create_timer(1.0 / scan_rate, self.publish_scan)
        self.create_timer(1.0, self.publish_obstacles)
        self.get_logger().info(
            f'World {width}x{height} m with {len(self.world.circles)} obstacles, '
            f'{len(self.world.segments)} walls')

    def on_cmd(self, msg):
        self.cmd = msg
        self.last_cmd = self.get_clock().now()

    # ------------------------------------------------------------ motion
    def step(self):
        now = self.get_clock().now()
        v, w = self.cmd.linear.x, self.cmd.angular.z
        if (now - self.last_cmd).nanoseconds > 5e8:
            v = w = 0.0
        new_theta = math.atan2(math.sin(self.theta + w * self.dt), math.cos(self.theta + w * self.dt))
        nx = self.x + v * math.cos(self.theta) * self.dt
        ny = self.y + v * math.sin(self.theta) * self.dt
        if self.world.collides(nx, ny, self.robot_radius):
            if v != 0.0:
                self.collisions += 1
                self.get_logger().warn(f'Bump! collisions so far: {self.collisions}',
                                       throttle_duration_sec=1.0)
            v = 0.0
        else:
            self.x, self.y = nx, ny
        self.theta = new_theta
        self.publish_state(now, v, w)

    def publish_state(self, now, v, w):
        qz, qw = yaw_quat(self.theta)
        tf = TransformStamped()
        tf.header.stamp = now.to_msg()
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_link'
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf.sendTransform(tf)

        odom = Odometry()
        odom.header = tf.header
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        self.odom_pub.publish(odom)

        m = Marker()
        m.header.frame_id = 'base_link'
        m.header.stamp = tf.header.stamp
        m.ns, m.id, m.type = 'robot', 0, Marker.CYLINDER
        m.scale.x = m.scale.y = 2 * self.robot_radius
        m.scale.z = 0.1
        m.pose.position.z = 0.05
        m.pose.orientation.w = 1.0
        m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.5, 1.0, 0.9
        self.robot_pub.publish(m)

    # ------------------------------------------------------------ sensors
    def publish_static_laser(self):
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'base_link'
        tf.child_frame_id = 'laser'
        tf.transform.translation.x = self.laser_offset
        tf.transform.rotation.w = 1.0
        self.static_tf.sendTransform(tf)

    def publish_scan(self):
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'laser'
        scan.angle_min = -math.pi
        scan.angle_increment = 2.0 * math.pi / self.beams
        scan.angle_max = scan.angle_min + scan.angle_increment * (self.beams - 1)
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.scan_time = 0.1
        lx = self.x + self.laser_offset * math.cos(self.theta)
        ly = self.y + self.laser_offset * math.sin(self.theta)
        ranges = []
        for i in range(self.beams):
            angle = self.theta + scan.angle_min + i * scan.angle_increment
            r = self.world.cast(lx, ly, angle, self.range_max)
            if r >= self.range_max:
                ranges.append(float('inf'))  # REP 117: no return
            else:
                ranges.append(max(self.range_min, r + random.gauss(0.0, self.noise)))
        scan.ranges = ranges
        self.scan_pub.publish(scan)

    def publish_obstacles(self):
        arr = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for i, (cx, cy, r) in enumerate(self.world.circles):
            m = Marker()
            m.header.frame_id = 'odom'
            m.header.stamp = stamp
            m.ns, m.id, m.type = 'circles', i, Marker.CYLINDER
            m.pose.position.x, m.pose.position.y, m.pose.position.z = cx, cy, 0.25
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = 2 * r
            m.scale.z = 0.5
            m.color.r, m.color.g, m.color.b, m.color.a = 0.8, 0.3, 0.2, 1.0
            arr.markers.append(m)
        walls = Marker()
        walls.header.frame_id = 'odom'
        walls.header.stamp = stamp
        walls.ns, walls.id, walls.type = 'walls', 0, Marker.LINE_LIST
        walls.scale.x = 0.05
        walls.pose.orientation.w = 1.0
        walls.color.r = walls.color.g = walls.color.b = 0.9
        walls.color.a = 1.0
        for (x1, y1), (x2, y2) in self.world.segments:
            walls.points += [Point(x=x1, y=y1), Point(x=x2, y=y2)]
        arr.markers.append(walls)
        self.marker_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = WorldSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
