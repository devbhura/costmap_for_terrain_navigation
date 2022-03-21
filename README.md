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


## Project Details
## 1. 3D SLAM 
Rtabmap is used to create a lidar map. Rtabmap used RGB-D SLAM approach, using the Velodyne Lidar PointCloud and Intel Realsense D435i camera to create a 3D PointCloud map. Creating an accurate lidar pointcloud map is used to create 2D occupancy map. 


## 2. PCL Clustering and Costmap2D 
Although rtabmap produces an occupancy map, it is not good enough for navigation as it has many grid points that are occupied due to noisy lidar data. Therefore clustering of objects based on rtabmap pointcloud gives a better sense of where large obstacles are. Using this clustering, a costmap is created where small obstales over which the jackal can go over is shown as available, while only large obstacles are make the costmap region occupied. 

## Demo

RVIZ With original map:
<iframe width="560" height="315" src="https://www.youtube.com/embed/mh6tp0GCJg8" title="YouTube video player" frameborder="0" allow="" allowfullscreen></iframe>
                    
Real Robot with original map:

<iframe width="560" height="315" src="https://www.youtube.com/embed/0GyZ2B7ILJ8" title="YouTube video player" frameborder="0" allow="" allowfullscreen></iframe>

RVIZ with custom costmap:
<iframe width="560" height="315" src="https://www.youtube.com/embed/Fy4MezXpsys" title="YouTube video player" frameborder="0" allow="" allowfullscreen></iframe>

Real robot going over obstacle :
<iframe width="560" height="315" src="https://www.youtube.com/embed/tTjn8FdkOCQ" title="YouTube video player" frameborder="0" allow="" allowfullscreen></iframe>
