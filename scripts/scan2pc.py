#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
import message_filters

import numpy as np
from matplotlib import pyplot as plt
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan, PointCloud2
import laser_geometry.laser_geometry as lg
# import tf listener
import tf2_ros

from rclpy.qos import qos_profile_sensor_data
class TrackedObjects:
    def __init__(self, node):
        self.node = node
        self.tracked_objects = []
        self.timestamp = None
        # set qos profile reliability


        self.qos_profile = qos_profile_sensor_data

        self.laser_back_sub = node.create_subscription(LaserScan, 'scan', self.laser_callback, qos_profile_sensor_data)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, node)
        self.point2_pub = node.create_publisher(PointCloud2, 'point_cloud', 1)
    
    
    def laser_callback(self, msg):
        lp = lg.LaserProjection()
        pc = lp.projectLaser(msg)

        # publish point cloud
        self.point2_pub.publish(pc)

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('scan2pc')

    scan2pc = TrackedObjects(node)

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
