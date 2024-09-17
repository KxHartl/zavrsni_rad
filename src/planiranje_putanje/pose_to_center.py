import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from tf_transformations import quaternion_from_euler
from tf_transformations import euler_from_quaternion
import math
import numpy as np


class FramePublisher(Node):
    def __init__(self):
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
)

        super().__init__('pose_to_centre')
        self.subscriptionOptiTrack = self.create_subscription(
            PoseStamped,
            '/vrpn_mocap/astro/pose',
            self.handle_optitrack,
            qos_profile 
            )
        self.subscriptionOptiTrack

        self.subscriptioMarvelmind = self.create_subscription(
            PoseStamped,
            '/marvelmind_pose',
            self.handle_marvelmind,
            10
        )
        self.subscriptioMarvelmind

        self.publisherOptiTrackCenter = self.create_publisher(
            PoseStamped,
            '/optitrack_center',
            10
        )

        self.publisherMarvelmindCenter = self.create_publisher(
            PoseStamped,
            '/marvelmind_center',
            10
        )

    def handle_optitrack(self, msg):
        x_shift = -0.03
        y_shift = 0.00
        
        theta = euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])

        # x_c = msg.pose.position.x + 0.015
        # y_c = msg.pose.position.y
    
        # Transformacija pozicije
        

        x_c = msg.pose.position.x - 0.005 * np.cos(theta[2]) 
        y_c = msg.pose.position.y - 0.005 * np.sin(theta[2])

        x_new = msg.pose.position.x - x_c
        y_new = msg.pose.position.y - y_c

        # Primjena rotacije
        # x_c = msg.pose.position.x * np.cos(theta[2]) + msg.pose.position.y * np.sin(theta[2])
        # y_c = msg.pose.position.x * -1 * np.sin(theta[2]) + msg.pose.position.y * np.cos(theta[2])

        p = PoseStamped()
        p.header.stamp = self.get_clock().now().to_msg()
        p.header.frame_id = 'world'
        p.pose.position.x = x_new
        p.pose.position.y = y_new
        p.pose.position.z = msg.pose.position.z
        p.pose.orientation.x = msg.pose.orientation.x
        p.pose.orientation.y = msg.pose.orientation.y
        p.pose.orientation.z = msg.pose.orientation.z
        p.pose.orientation.w = msg.pose.orientation.w

        self.publisherOptiTrackCenter.publish(p)

        
    def handle_marvelmind(self, msg):
        pass
        

def main():
    rclpy.init()
    node = FramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()