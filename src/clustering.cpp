#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <rtab_slam/SegmentedClustersArray.h>
#include <Eigen/Eigen>
#include <pcl/features/normal_3d.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>


class segmentation {

public:

  explicit segmentation(ros::NodeHandle nh) : m_nh(nh)  {

    // define the subscriber and publisher 
    // m_sub = m_nh.subscribe ("/mid/points", 1, &segmentation::cloud_cb, this);
    m_sub = m_nh.subscribe ("/rtabmap/cloud_obstacles", 1, &segmentation::cloud_cb, this);
    m_clusterPub = m_nh.advertise<sensor_msgs::PointCloud2> ("obj_recognition/pcl_clusters",1);
    costmap_cloudPub = m_nh.advertise<sensor_msgs::PointCloud> ("obj_recognition/costmap_cloud",1);
    m_bounding = m_nh.advertise<visualization_msgs::MarkerArray>("obj_recognition/bounding_box",1, true);
  }

private:

ros::NodeHandle m_nh;
ros::Publisher m_pub;
ros::Subscriber m_sub;
ros::Publisher m_clusterPub;
ros::Publisher m_bounding;
ros::Publisher costmap_cloudPub; 

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

}; // end class definition


// define callback function
void segmentation::cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{



  // Container for original & filtered data
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);


  // Convert to PCL data type
  pcl::fromROSMsg(*cloud_msg, *cloud);


  // Perform voxel grid downsampling filtering
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (0.02f, 0.02f, 0.02f);
  sor.filter (*cloud_filtered);

  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PCDWriter writer;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.02);

  int nr_points = (int) cloud_filtered->size ();
  while (cloud_filtered->size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);
    std::cout << "PointCloud representing the planar component: " << cloud_plane->size () << " data points." << std::endl;

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *cloud_filtered = *cloud_f;
  }

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.2); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (2500);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);

  // // declare an instance of the SegmentedClustersArray message
  // rtab_slam::SegmentedClustersArray CloudClusters;

  int r = 255;
  int g = 255;
  int b = 255;
  int ctr = 0;

  visualization_msgs::MarkerArray bounding_box;

  // // declare the output variable instances
  sensor_msgs::PointCloud2::Ptr clusters (new sensor_msgs::PointCloud2);
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr clusterPtr (new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<pcl::PointIndices>::const_iterator it;
  std::vector<int>::const_iterator pit;
  for(it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
          pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
          for(pit = it->indices.begin(); pit != it->indices.end(); pit++) {
          //push_back: add a point to the end of the existing vector
                  cloud_cluster->points.push_back(cloud_filtered->points[*pit]); 
          }

        Eigen::Vector4f centroid;
        Eigen::Vector4f min;
        Eigen::Vector4f max;
        pcl::getMinMax3D(*cloud_cluster, min, max);

        pcl::compute3DCentroid(*cloud_cluster, centroid);
        std::cout<<"In for loop"<<std::endl; 
        std::cout<<"centroid"<< centroid[0] <<std::endl; 
        std::cout<<"min"<< min <<std::endl; 
        std::cout<<"max"<< max <<std::endl; 
        //Merge current clusters to whole point cloud

      

      *clusterPtr += *cloud_cluster;

      visualization_msgs::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = ros::Time::now();

      marker.id = ctr;
      marker.type = visualization_msgs::Marker::CUBE;;
      marker.action = visualization_msgs::Marker::ADD;

      marker.pose.position.x = centroid[0];
      marker.pose.position.y = centroid[1];
      marker.pose.position.z = centroid[2];
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;

      marker.scale.x = (max[0]-min[0]);
      marker.scale.y = (max[1]-min[1]);
      marker.scale.z = (max[2]-min[2]);

      if (marker.scale.x ==0)
          marker.scale.x=0.1;

      if (marker.scale.y ==0)
        marker.scale.y=0.1;

      if (marker.scale.z ==0)
        marker.scale.z=0.1;

      marker.color.r = r;
      marker.color.g = g;
      marker.color.b = b;
      marker.color.a = 0.5;

      bounding_box.markers.push_back(marker);

      ctr++;
      if(ctr%2==0)
      {
        r--;
        if(r<0){r=255;}
      }

      if(ctr%3==0)
      {
        g--;
        if(g<0){g=255;}
      }

      if(ctr%5==0)
      {
        b--;
        if(b<0){b=255;}
      }


  }
  pcl::toROSMsg (*clusterPtr , *clusters);
  
  clusters->header.frame_id = "map";
  clusters->header.stamp = ros::Time::now();
  
    // publish the clusters
  m_clusterPub.publish(*clusters);

  m_bounding.publish(bounding_box);

  sensor_msgs::PointCloud::Ptr clusters_pc (new sensor_msgs::PointCloud);
  clusters_pc->header.frame_id = "map";
  clusters_pc->header.stamp = ros::Time::now();

  sensor_msgs::convertPointCloud2ToPointCloud(*clusters,*clusters_pc); 

  costmap_cloudPub.publish(*clusters_pc); 

}



int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "segmentation");
  ros::NodeHandle nh;

  segmentation segs(nh);

  while(ros::ok())
  ros::spin ();

}