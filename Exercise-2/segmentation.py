#!/usr/bin/env python

# Import modules
from pcl_helper import *
import rospy
# TODO: Define functions as required

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

    # TODO: Convert ROS msg to PCL data

    # TODO: Voxel Grid Downsampling

    # TODO: PassThrough Filter


    # TODO: RANSAC Plane Segmentation

    # TODO: Extract inliers and outliers

    # TODO: Euclidean Clustering

    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately

    # TODO: Convert PCL data to ROS messages

    # TODO: Publish ROS messages
    pcl_objects_pub.publish(pcl_msg)
    pcl_table_pub.publish(pcl_msg)

if __name__ == '__main__':

    # TODO: ROS node initialization
    rospy.init_node('clustering', anonymous=True)

    # TODO: Create Subscribers
    pcl_sub = rospy.Subscriber("/sensor_stick/point_cloud", pc2.PointCloud2, pcl_callback, queue_size=1)

    # TODO: Create Publishers
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)

    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
    # Initialize color_list
    get_color_list.color_list = []
    
    # TODO: Spin while node is not shutdown
    rospy.spin()
