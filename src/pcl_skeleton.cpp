#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

ros::Publisher pub;
// How to avoid hard-coding a topic name?

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input) {
  // Create a container for the data.
  sensor_msgs::PointCloud2 output;

  /* Do data processing here...*/
  output = *input; // modify input

  // Publish the data.
  pub.publish(output);
}

int main (int argc, char** argv) {
  // Initialize ROS
  ros::init (argc, argv, "pcl_skeleton");
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
