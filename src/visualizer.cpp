#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Pose.h> // Eigen visualization
#include <geometry_msgs/PoseStamped.h> // Eigen visualization
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

ros::Publisher pub_marker;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input) {
  std::cerr << "in cloud_cb" << std::endl;
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  // pcl::PCLPointCloud2 cloud_filtered;
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_templated (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
  pcl_conversions::toPCL(*input, *cloud);

  
}

int main (int argc, char** argv) {

  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("cloud_pcd", 1, cloud_cb);

  pub_p = nh.advertise<sensor_msgs::PointCloud2>("output_p", 1);
  pub_r = nh.advertise<sensor_msgs::PointCloud2>("cloud_rot", 1);
  pub_red = nh.advertise<sensor_msgs::PointCloud2>("cloud_red", 1);
  pub_m = nh.advertise<visualization_msgs::Marker>("marker", 1, 0);

  ros::spin();
}
