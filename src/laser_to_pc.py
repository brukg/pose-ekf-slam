#!/usr/bin/python3

import rospy
from sensor_msgs.msg import PointCloud2, LaserScan
import laser_geometry.laser_geometry as lg


pc_pub = rospy.Publisher(rospy.get_param('/laser_to_pc/rplidar_pc'), PointCloud2, queue_size=10)

def scan_cb(msg):
    lp = lg.LaserProjection()
    pc2_msg = lp.projectLaser(msg)
    pc_pub.publish(pc2_msg)    

rospy.init_node("laser_to_pc")
rospy.Subscriber(rospy.get_param('/laser_to_pc/rplidar_scan'), LaserScan, scan_cb, queue_size=10)
rospy.spin()