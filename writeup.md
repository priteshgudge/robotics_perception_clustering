## Project: Robotics Perception 

---
**Steps to Run the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/priteshgudge/robotics_perception_clustering/) into the ***src*** directory of your ROS Workspace.  
3. Use the template file to generate your script.
4. Run catkin_make to build the project.
5. Setup the GAZEBO PATH and source in bashrc: 
`export GAZEBO_MODEL_PATH=~/catkin_ws/src/sensor_stick/models`
`source ~/catkin_ws/devel/setup.bash`
5. Fill in the code into segmentation.py in scripts
6. Run the project with `roslaunch sensor_stick robot_spawn.launch`
7. Run the segmentation.py file seperately. `python segmentation.py`
8. Change the Topic in the Point Cloud Display on Rviz to iterate through the various pointclouds published on topics. 

[//]: # "Image References"


[perception_rviz]: ./images/perception_rviz.png
[pc_table]: ./images/pcl_table.png
[pc_objects]: ./images/pcl_objects.png
[clustering]: ./images/pcl_cluster1.png


---
### Launching the project
#### 1
The camera topic default view in Rviz.

![alt text][perception_rviz]

#### 2
After publishing to the table pointcloud topic and appropriate selection in Rviz

![alt text][pc_table]

#### 3
After publishing to the objects pointcloud topic and appropriate selection in Rviz

![alt text][pc_table]

#### 4
When the clustering algorithm is integrated and the result publised on the /pcl_cluster topic and the topic is subscribed in Rviz.

![alt text][cluster]

