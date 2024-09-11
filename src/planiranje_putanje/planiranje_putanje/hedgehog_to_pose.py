from marvelmind_ros2_msgs.msg import HedgePositionAngle
from marvelmind_ros2_msgs.msg import BeaconPositionAddressed
import rclpy
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import PoseStamped

from tf_transformations import quaternion_from_euler
import math
import numpy as np
import time


class FramePublisher(Node):
    def __init__(self):
        super().__init__('marvelmind_to_pose')

        self.angles = []
        self.angle_diff = 0.0
        self.angle_corected = False

        self.count = 0
        self.last_angle = 0.0

        self.tf_broadcaster = TransformBroadcaster(self)

        self.subscriptionHedge = self.create_subscription(
            HedgePositionAngle,
            '/hedgehog_pos_ang',
            self.handle_hedgehog_pos_ang,
            10
            )
        self.subscriptionHedge

        self.subscriptionBeacons = self.create_subscription(
            BeaconPositionAddressed,
            '/beacons_pos_addressed',
            self.handle_beacons_pos_addressed,
            10
        )
        self.subscriptionBeacons

        self.publisherHedgePose = self.create_publisher(
            PoseStamped,
            '/marvelmind_pose',
            10
        )

    def handle_hedgehog_pos_ang(self, msg):
        self.count = self.count + 1

        if not self.angle_corected:
            self.angles.append(msg.angle)
            if len(self.angles) > 20:
                self.angles.sort
                angles_med = self.angles[9]
                self.angle_diff = -angles_med
                self.angle_corected = True

        x_new = msg.x_m
        y_new = msg.y_m

        angle_drift = self.count * 0.00165/21

        self.last_angle = math.radians(msg.angle + self.angle_diff) - angle_drift

        q = quaternion_from_euler(0, 0, math.radians(msg.angle + self.angle_diff) - angle_drift)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'hedgehog'
        t.transform.translation.x = x_new
        t.transform.translation.y = y_new
        t.transform.translation.z = msg.z_m
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        p = PoseStamped()
        p.header.stamp = self.get_clock().now().to_msg()
        p.header.frame_id = 'world'
        p.pose.position.x = x_new
        p.pose.position.y = y_new
        p.pose.position.z = msg.z_m
        p.pose.orientation.x = q[0]
        p.pose.orientation.y = q[1]
        p.pose.orientation.z = q[2]
        p.pose.orientation.w = q[3]

        self.tf_broadcaster.sendTransform(t)
        self.publisherHedgePose.publish(p)

        
    def handle_beacons_pos_addressed(self, msg):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = str(msg.address)

        t.transform.translation.x = msg.x_m
        t.transform.translation.y = msg.y_m
        t.transform.translation.z = msg.z_m

        q = quaternion_from_euler(0, 0, 0)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = FramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
