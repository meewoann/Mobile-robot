#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import math


class OdomPublisher(Node):
    def __init__(self):
        super().__init__('odom_publisher')

        # ===== ROS interfaces =====
        self.cmd_sub = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.cmd_callback,
            10
        )

        self.odom_pub = self.create_publisher(
            Odometry,
            "/odom",
            10
        )

        self.br = TransformBroadcaster(self)

        # ===== Odom state =====
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.vx = 0.0
        self.vth = 0.0

        self.last_time = self.get_clock().now()

        # 20 Hz
        self.timer = self.create_timer(0.05, self.publish_odom)

    def cmd_callback(self, msg: Twist):
        # Lưu velocity command
        self.vx = msg.linear.x
        self.vth = msg.angular.z

    def publish_odom(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        if dt <= 0.0:
            return

        # ===== Integrate motion =====
        if abs(self.vx) < 1e-3 and abs(self.vth) > 1e-3:
            # ---- Rotate in place ----
            self.th += self.vth * dt
        else:
            # ---- Normal motion ----
            self.x += self.vx * math.cos(self.th) * dt
            self.y += self.vx * math.sin(self.th) * dt
            self.th += self.vth * dt

        # Normalize yaw (-pi, pi)
        self.th = math.atan2(math.sin(self.th), math.cos(self.th))

        # ===== Quaternion (yaw only) =====
        qz = math.sin(self.th * 0.5)
        qw = math.cos(self.th * 0.5)

        # ===== TF: odom -> base_link =====
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.br.sendTransform(t)

        # ===== Odometry message =====
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = self.vth

        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = OdomPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
