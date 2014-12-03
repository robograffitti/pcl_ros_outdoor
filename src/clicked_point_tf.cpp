#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>

ros::Publisher pub;

void clicked_cb(const geometry_msgs::PointStamped& clicked_pt) {
  
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "clicked_point_tf");
  ros::NodeHandle nh;
  ros::Subscriber sub= nh.subscribe("clicked_point", 1, clicked_cb);
  pub = nh.advertise<tf2_msgs::TFMessage>("clicked_point_tf", 1);
  ros::Spin();
}
