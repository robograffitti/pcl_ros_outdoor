#include <ros/ros.h>
#include <ros/rosconsole.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

ros::Publisher pub;
// How to avoid hard-coding a topic name?

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input) {
  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;

  // Convert to PCL data type
  pcl_conversions::toPCL(*input, *cloud);
  // Fill in the cloud data
  ROS_DEBUG_STREAM("cloud_width:" << cloud->width);

  /* Perform the actual filtering */
  /* pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud(cloudPtr);
  sor.setLeafSize(0.1, 0.1, 0.1);
  sor.filter(cloud_filtered); */

  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  output = *input; // only for debuging
  // pcl_conversions::fromPCL(cloud_filtered, output);

  // Publish the data.
  pub.publish(output);
}

int main (int argc, char** argv) {
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;
  
  // Create a ROS subscriber for the input point cloud
  // when topic /input is incoming, cloud_cb callback is called
  ros::Subscriber sub = nh.subscribe("assemble_cloud", 1, cloud_cb);
  
  // Create a ROS publisher for the output point cloud
  // A node has both of publisheres and subscribers.
  pub = nh.advertise<sensor_msgs::PointCloud2>("output", 1);

  // Spin
  ros::spin();
}
