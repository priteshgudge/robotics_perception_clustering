#!/usr/bin/env python

# Import modules
from pcl_helper import *
import rospy
# TODO: Define functions as required

def voxel_grid_filter(cloud_obj):
    vox = cloud_obj.make_voxel_grid_filter()

    # Choose a voxel (also known as leaf) size
    # Note: this (1) is a poor choice of leaf size   
    # Experiment and find the appropriate size!
    LEAF_SIZE = 0.01

    # Set the voxel (or leaf) size  
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)

    # Call the filter function to obtain the resultant downsampled point cloud
    cloud_filtered = vox.filter()
    
    return cloud_filtered
    
def passthrough_filter(cloud_obj):

    passthrough = cloud_obj.make_passthrough_filter()

    # Assign axis and range to the passthrough filter object.
    filter_axis = 'z'
    passthrough.set_filter_field_name (filter_axis)
    axis_min = 0.6
    axis_max = 1.1
    passthrough.set_filter_limits (axis_min, axis_max)

    # Finally use the filter function to obtain the resultant point cloud. 
    cloud_filtered = passthrough.filter()
    
    return cloud_filtered

def ransac_plane_segmentation(cloud_obj):

    # Create the segmentation object
    seg = cloud_obj.make_segmenter()

    # Set the model you wish to fit 
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)

    # Max distance for a point to be considered fitting the model
    # Experiment with different values for max_distance 
    # for segmenting the table
    max_distance = 0.01
    seg.set_distance_threshold(max_distance)
    

    return seg

def convert_to_kd_tree(object_cloud):
    white_cloud = XYZRGB_to_XYZ(object_cloud)# Apply function to convert XYZRGB to XYZ
    tree = white_cloud.make_kdtree()
    return tree, white_cloud

def cluster_extraction_from_kd_tree(kd_tree, white_cloud):
    # Create a cluster extraction object
    ec = white_cloud.make_EuclideanClusterExtraction()
    # Set tolerances for distance threshold 
    # as well as minimum and maximum cluster size (in points)
    # NOTE: These are poor choices of clustering parameters
    # Your task is to experiment and find values that work for segmenting objects.
    ec.set_ClusterTolerance(0.001)
    ec.set_MinClusterSize(10)
    ec.set_MaxClusterSize(250)
    # Search the k-d tree for clusters
    ec.set_SearchMethod(kd_tree)
    # Extract indices for each of the discovered clusters
    cluster_indices = ec.Extract()
    #print cluster_indices
    return cluster_indices

def generate_cluster_cloud(cluster_indices, white_cloud):
    #Assign a color corresponding to each segmented object in scene
    #print cluster_indices
    cluster_color = get_color_list(len(cluster_indices))
    #print cluster_color

    color_cluster_point_list = []

    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                            white_cloud[indice][1],
                                            white_cloud[indice][2],
                                             rgb_to_float(cluster_color[j])])

    #Create new cloud containing all clusters, each with unique color
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)

    return cluster_cloud

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

    # TODO: Convert ROS mg to PCL data
    cloud = ros_to_pcl(pcl_msg)
  

    # TODO: Voxel Grid Downsampling
    vox = voxel_grid_filter(cloud)

    # TODO: PassThrough Filter
    
    cloud_filtered = passthrough_filter(vox)


    # TODO: RANSAC Plane Segmentation
    
    seg = ransac_plane_segmentation(cloud_filtered)
    inliers, coefficients = seg.segment()
    # TODO: Extract inliers and outliers

    extracted_inliers = cloud_filtered.extract(inliers, negative=False)
    extracted_outliers = cloud_filtered.extract(inliers, negative=True)
    
    cloud_table = extracted_inliers
    cloud_objects = extracted_outliers
    # TODO: Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(cloud_objects)# Apply function to convert XYZRGB to XYZ
    kd_tree = white_cloud.make_kdtree()

    #Create a cluster extraction object
    ec = white_cloud.make_EuclideanClusterExtraction()
    # Set tolerances for distance threshold 
    # as well as minimum and maximum cluster size (in points)
    # NOTE: These are poor choices of clustering parameters
    # Your task is to experiment and find values that work for segmenting objects.
    ec.set_ClusterTolerance(0.05)
    ec.set_MinClusterSize(100)
    ec.set_MaxClusterSize(5000)
    # Search the k-d tree for clusters
    ec.set_SearchMethod(kd_tree)
    # Extract indices for each of the discovered clusters
    cluster_indices = ec.Extract()
    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
    
    #cluster_cloud = generate_cluster_cloud(cluster_indices, white_cloud)
    #Assign a color corresponding to each segmented object in scene
    cluster_color = get_color_list(len(cluster_indices))
    print cluster_color
    color_cluster_point_list = []

    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                            white_cloud[indice][1],
                                            white_cloud[indice][2],
                                             rgb_to_float(cluster_color[j])])

    #Create new cloud containing all clusters, each with unique color
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)

    # TODO: Convert PCL data to ROS messages

    pcl_obj = pcl_to_ros(cloud_objects)
    pcl_table = pcl_to_ros(cloud_table)
    pcl_cluster_cloud = pcl_to_ros(cluster_cloud)
    #print(pcl_cluster_cloud)
    # TODO: Publish ROS messages
    pcl_objects_pub.publish(pcl_obj)
    pcl_table_pub.publish(pcl_table)
    pcl_cluster_pub.publish(pcl_cluster_cloud)

if __name__ == '__main__':

    # TODO: ROS node initialization
    rospy.init_node('clustering', anonymous=True)

    # TODO: Create Subscribers
    pcl_sub = rospy.Subscriber("/sensor_stick/point_cloud", pc2.PointCloud2, pcl_callback, queue_size=1)

    # TODO: Create Publishers
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)

    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)

    pcl_cluster_pub = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size=1)

    # Initialize color_list
    get_color_list.color_list = []
    
    # TODO: Spin while node is not shutdown
    rospy.spin()
