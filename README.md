# 3D SLAM Navigation and Obstacle threshold Mapping for Optimal Terrain Navigation

## Devesh Bhura Individual Winter Project 

## Description 
The repository implements 3D SLAM package, rtabmap, for navigation of the jackal robot. The project clusters obstacles and creates a costmap that allows move base to move over small obstacles. 

## Dependencies 
### Hardware:
* Clearpath Jackal Robot
* Velodyne-16 lidar
* Intel Realsense Camera D435i

### Software:
ROS Noetic, Rviz, Gazebo, rtabmap, costmap2d, PCL

## Jackal Bringup 
Download the repo on both the jackal workspace as well on your own laptop. 

Before running all the nodes on the jackal, `source src/rtab_slam/setup_jackal.bash` on the jackal and `source src/rtab_slam/setup_pc.bash` on the local pc. This must be done for every new terminal window. This ensures that nodes are being run on the same ROS_MASTER_URI. 

To start 3D slam, run `roslaunch rtab_slam jackal_bringup.launch` on the jackal. This starts rtabmap. To create a 2D costmap, run move base and clustering algorithm, run `roslaunch rtab_slam real_robot.launch` on the pc. 

## Demo 


## Project Details
## 1. 3D SLAM 
Rtabmap is used to map 

## 2. PCL Clustering and Costmap2D 