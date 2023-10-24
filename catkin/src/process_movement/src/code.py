#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
import pcl
import pcl.pcl_visualization
import pcl.pcl_io
import pcl.pcl_registration
import pcl.pcl_filter
import pcl.pcl_segmentation
from pcl import PointCloud


def object_callback(point_cloud_msg):
    # Convert the PointCloud2 message to a PCL PointCloud
    pcl_cloud = pcl.PointCloud()
    pcl_cloud.from_list(point_cloud_msg.data)

    # Process the point cloud data to detect and avoid objects
    # Your object detection and avoidance logic should go here

    # For demonstration purposes, let's just print the number of points in the cloud
    num_points = pcl_cloud.size
    rospy.loginfo(f"Received point cloud data with {num_points} points.")

    # Your avoidance logic would depend on the specific use case and sensor setup


def my_node():
    rospy.init_node('process_movement')

    # Subscribe to the object_pub_ topic to get point cloud data
    rospy.Subscriber('lidar/object_pub_', PointCloud2, object_callback)

    # Spin to keep the node running
    rospy.spin()


if __name__ == '__main__':
    my_node()
