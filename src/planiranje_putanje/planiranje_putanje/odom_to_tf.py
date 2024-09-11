import rclpy
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

from tf_transformations import quaternion_from_euler
import math
import numpy as np


class FramePublisher(Node):
    def __init__(self):
        super().__init__('odom_to_tf')

        self.is_start = True

        self.odom_start = [None, None, None, None, None, None, None]

        self.tf_broadcaster = TransformBroadcaster(self)

        self.subscriptionOdom = self.create_subscription(
            Odometry,
            '/odom',
            self.handle_odom,
            1
            )
        self.subscriptionOdom

    def handle_odom(self, msg):
        if self.is_start:
            self.odom_start[0] = msg.pose.pose.position.x
            self.odom_start[1] = msg.pose.pose.position.y
            self.odom_start[2] = msg.pose.pose.position.z
            self.odom_start[3] = msg.pose.pose.orientation.x
            self.odom_start[4] = msg.pose.pose.orientation.y
            self.odom_start[5] = msg.pose.pose.orientation.z
            self.odom_start[6] = msg.pose.pose.orientation.w

            self.is_start = False

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'odom'

        t.transform.translation.x = self.odom_start[0]
        t.transform.translation.y = self.odom_start[1]
        t.transform.translation.z = self.odom_start[2]
        t.transform.rotation.x = self.odom_start[3]
        t.transform.rotation.y = self.odom_start[4]
        t.transform.rotation.z = self.odom_start[5]
        t.transform.rotation.w = self.odom_start[6]
        self.tf_broadcaster.sendTransform(t)

        b = TransformStamped()
        b.header.stamp = self.get_clock().now().to_msg()
        b.header.frame_id = 'odom'
        b.child_frame_id = 'base_footprint'

        b.transform.translation.x = msg.pose.pose.position.x
        b.transform.translation.y = msg.pose.pose.position.y
        b.transform.translation.z = msg.pose.pose.position.z
        b.transform.rotation.x = msg.pose.pose.orientation.x
        b.transform.rotation.y = msg.pose.pose.orientation.y
        b.transform.rotation.z = msg.pose.pose.orientation.z
        b.transform.rotation.w = msg.pose.pose.orientation.w

        self.tf_broadcaster.sendTransform(b)
        

def main():
    rclpy.init()
    node = FramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()