#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
// #include <tf2_msgs/TFMessage.h>
#include <tf/transform_broadcaster.h>

ros::Publisher pub;

void clicked_cb(const geometry_msgs::PointStampedConstPtr& msg) {
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(msg->point.x, msg->point.y, msg->point.z));
  tf::Quaternion q;
  q.setRPY(0, 0, 0);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, msg->header.stamp, "world","clicked_point"));
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "clicked_point_tf");
  ros::NodeHandle nh;
  ros::Subscriber sub= nh.subscribe("clicked_point", 1, &clicked_cb);

  // pub = nh.advertise<tf2_msgs::TFMessage>("clicked_point_tf", 1);
  ros::spin();
  // return 0;
};
