#include <iostream>
#include <sstream>
// using namespace std; // namespace for std::string and etc.
#include <ros/ros.h>
#include <ros/console.h>

// ROS msgs
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PolygonStamped.h>
#include <visualization_msgs/Marker.h>
#include <jsk_recognition_msgs/PointsArray.h> // point cloud array
#include <jsk_recognition_msgs/PolygonArray.h>

// PCL related
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/pca.h>
// #include <pcl-1.7/pcl/common/angles.h>
#include <pcl/common/angles.h>

ros::Publisher pub_voxel; // voxel cloud, pub_voxel
ros::Publisher pub_plane; // plane cloud, pub_planelane
ros::Publisher pub_rot; // rotated cloud, pub_rot
ros::Publisher pub_red; // reduced cloud, pub_red
ros::Publisher pub_marker;
ros::Publisher pub_polygon_array;

// Global Variables
// 1. How to avoid hard-coding a topic name? (use parameters, etc.)
// 2. For research, refering to existing equation,
// explain how parameter value is defined.
// 3. Use nodelet to devide this process into threads
// Replace this function with SVM linear division (or SegmentDifferences?)
// rename divide to reduce... ?

// Separate into separate clouds and publish polygons
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > // use jsk_recognition_msgs::PointsArray
separate(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz_rot, std_msgs::Header header) {
  double x_pitch = 0.25, x_min = 1.0, x_max = 3.0; // 1.5~1.75 1.75~2.00 1.5~1.675
  double y_min = -0.75, y_max = 0.75;
  double z_min = -0.250, z_1 = 0.000, z_2 = 1.000, z_max = 1.750; // -0.3125, 2.0
  pcl::PointXYZ pt_1, pt_2, pt_3, pt_4, pt_5, pt_6; // deprecate with polygon

  // Divide large cloud
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > cloud_vector;
  // pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::PointXYZ tmp_p;

  jsk_recognition_msgs::PolygonArray polygon_array;
  polygon_array.header = header;
  for (int i = 0; i < (int)( (x_max - x_min) / x_pitch ); i++) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    geometry_msgs::PolygonStamped polygon;
    visualization_msgs::Marker texts, line_strip; // TEXT_VIEW_FACING

    texts.header = header;
    texts.ns = "text"; // namespace + ID
    texts.action = visualization_msgs::Marker::ADD;
    texts.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

    texts.pose.orientation.x = 0.0;
    texts.pose.orientation.y = 0.0;
    texts.pose.orientation.z = 0.0;
    texts.pose.orientation.w = 1.0;
    texts.scale.x = 0.125;
    texts.scale.y = 0.125;
    texts.scale.z = 0.125;
    texts.color.r = 1.0f;
    texts.color.g = 0.0f;
    texts.color.b = 0.0f;
    texts.color.a = 1.0;

    geometry_msgs::Point32 tmp_p_up_0, tmp_p_up_1, tmp_p_up_2, tmp_p_down_0, tmp_p_down_1, tmp_p_down_2;
    pcl::PointXYZ tmp_p;
    double width_tmp, width_min_up = 2.000, width_min_down = 4.000;
    double width_min_bottom = 0.500, width_min_top = 0.200;
    for (pcl::PointCloud<pcl::PointXYZ>::const_iterator itr = cloud_xyz_rot->begin();
         itr != cloud_xyz_rot->end(); itr++) {
      if ( (x_min + i*x_pitch) < itr->x && itr->x < (x_min + (i+1)*x_pitch) ) {
        if (y_min < itr->y && itr->y < y_max) {
          if (z_min < itr->z && itr->z < z_max) {
            // compare tmp_p and itr, and calculate width and points
            if (itr != cloud_xyz_rot->begin()) { // skip at 1st time
              if ( (tmp_p.y < 0 && 0 <= itr->y) || (itr->y < 0 && 0 <= tmp_p.y) ) {
                if (itr->z < z_1) {
                  width_tmp = sqrt(pow(fabs(tmp_p.x - itr->x), 2)
                                   + pow(fabs(tmp_p.y - itr->y), 2)
                                   + pow(fabs(tmp_p.z - itr->z), 2));
                  if (width_min_bottom < width_tmp && width_tmp <= width_min_down) {
                    width_min_down = width_tmp; // create width_min array
                    tmp_p_down_0.x = tmp_p.x; tmp_p_down_0.y = tmp_p.y;
                    tmp_p_down_0.z = (tmp_p.z + itr->z) / 2;
                    tmp_p_down_1.x = itr->x; tmp_p_down_1.y = itr->y;
                    tmp_p_down_1.z = (tmp_p.z + itr->z) / 2;
                    tmp_p_down_2.x = tmp_p.x; // ignore adding sqrt
                    tmp_p_down_2.y = tmp_p.y + sqrt(pow(fabs(tmp_p.y - itr->y), 2)) / 2;
                    tmp_p_down_2.z = (tmp_p.z + itr->z) / 2;
                  }
                }
                if (z_2 < itr->z) {
                  width_tmp = sqrt(pow(fabs(tmp_p.x - itr->x), 2)
                                   + pow(fabs(tmp_p.y - itr->y), 2)
                                   + pow(fabs(tmp_p.z - itr->z), 2));
                  if (width_tmp <= width_min_down) {
                    width_min_up = width_tmp;
                    tmp_p_up_0.x = tmp_p.x; tmp_p_up_0.y = tmp_p.y;
                    tmp_p_up_0.z = (tmp_p.z + itr->z) / 2;
                    tmp_p_up_1.x = itr->x; tmp_p_up_1.y = itr->y;
                    tmp_p_up_1.z = (tmp_p.z + itr->z) / 2;
                    tmp_p_up_2.x = tmp_p.x; // ignore adding sqrt
                    tmp_p_up_2.y = tmp_p.y + sqrt(pow(fabs(tmp_p.y - itr->y), 2)) / 2;
                    tmp_p_up_2.z = (tmp_p.z + itr->z) / 2;
                  }
                }
              }
              tmp_p.x = itr->x; tmp_p.y = itr->y; tmp_p.z = itr->z;
              tmp_cloud->points.push_back(tmp_p);
            }
          }
        }
      }
      // From tmp_cloud, get 4 points to publish marker
      // Create polygon
    }

    cloud_vector.push_back(tmp_cloud);

    tmp_p_up_0.x = x_min + i*x_pitch - x_pitch/2;
    tmp_p_up_1.x = x_min + i*x_pitch - x_pitch/2;
    tmp_p_down_0.x = x_min + i*x_pitch - x_pitch/2;
    tmp_p_down_1.x = x_min + i*x_pitch - x_pitch/2;
    if (tmp_p_up_0.y < tmp_p_up_1.y) {
      polygon.polygon.points.push_back(tmp_p_up_0);
      polygon.polygon.points.push_back(tmp_p_up_1);
    }
    if (tmp_p_up_0.y >= tmp_p_up_1.y) {
      polygon.polygon.points.push_back(tmp_p_up_1);
      polygon.polygon.points.push_back(tmp_p_up_0);
    }
    if (tmp_p_down_0.y < tmp_p_down_1.y) {
      polygon.polygon.points.push_back(tmp_p_down_1);
      polygon.polygon.points.push_back(tmp_p_down_0);
    }
    if (tmp_p_down_0.y >= tmp_p_down_1.y) {
      polygon.polygon.points.push_back(tmp_p_down_0);
      polygon.polygon.points.push_back(tmp_p_down_1);
    }
    polygon.header = header;
    polygon_array.polygons.push_back(polygon);

    std::cerr << "count:" << i << ", " << "size:" << cloud_vector.at(i)->size() << std::endl;
    std::cerr << "width_min_up:" << width_min_up << std::endl;
    std::cerr << "width_min_down:" << width_min_down << std::endl;

    texts.id = 2*i;
    texts.pose.position.x = tmp_p_up_0.x;
    texts.pose.position.y = tmp_p_up_2.y;
    texts.pose.position.z = tmp_p_up_2.z;

    std::ostringstream strs;
    strs << width_min_up;
    std::string str = strs.str();
    texts.text = str;
    pub_marker.publish(texts);

    texts.id = 2*i + 1;
    texts.pose.position.x = tmp_p_down_0.x;
    texts.pose.position.y = tmp_p_down_2.y;
    texts.pose.position.z = tmp_p_down_2.z;

    strs.str("");
    strs.clear(std::stringstream::goodbit);
    strs << width_min_down;
    str = strs.str();
    texts.text = str;
    pub_marker.publish(texts);

  }
  pub_polygon_array.publish(polygon_array); // error
  return cloud_vector;
}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &input) {
  // std::cerr << "in cloud_cb" << std::endl;
  /* 0. Importing input cloud */
  std_msgs::Header header = input->header;
  // std::string frame_id = input->header.frame_id;
  // sensor_msgs::PointCloud2 input_cloud = *input;

  pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2; // initialize object
  pcl_conversions::toPCL(*input, *cloud); // from input, generate content of cloud

  /* 1. Downsampling and Publishing voxel */
  // LeafSize: should small enough to caputure a leaf of plants
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud); // imutable pointer
  pcl::PCLPointCloud2 cloud_voxel; // cloud_filtered to cloud_voxel
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;

  sor.setInputCloud(cloudPtr); // set input
  sor.setLeafSize(0.02f, 0.02f, 0.02f); // 2cm, model equation
  sor.filter(cloud_voxel); // set output

  sensor_msgs::PointCloud2 output_voxel;
  pcl_conversions::fromPCL(cloud_voxel, output_voxel);
  pub_voxel.publish(output_voxel);

  /* 2. Filtering with RANSAC */
  // RANSAC initialization
  pcl::SACSegmentation<pcl::PointXYZ> seg; // Create the segmentation object
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  seg.setOptimizeCoefficients(true); // Optional
  // seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE); // Use SACMODEL_PERPENDICULAR_PLANE instead
  seg.setMethodType(pcl::SAC_RANSAC);

  // minimum number of points calculated from N and distanceThres
  seg.setMaxIterations (1000); // N in RANSAC
  seg.setDistanceThreshold(0.025); // 0.01 - 0.05 default: 0.02 // 閾値（しきい値）

  // param for perpendicular
  seg.setAxis(Eigen::Vector3f (0.0, 0.0, 1.0));
  seg.setEpsAngle(pcl::deg2rad (5.0f));

  // convert from PointCloud2 to PointXYZ
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel_xyz (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(cloud_voxel, *cloud_voxel_xyz);

  // RANSAC application
  seg.setInputCloud(cloud_voxel_xyz);
  seg.segment(*inliers, *coefficients); // values are empty at beginning

  // inliers.indices have array index of the points which are included as inliers
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane_xyz (new pcl::PointCloud<pcl::PointXYZ>);
  for (std::vector<int>::const_iterator pit = inliers->indices.begin ();
       pit != inliers->indices.end (); pit++) {
    cloud_plane_xyz->points.push_back(cloud_voxel_xyz->points[*pit]);
  }
  // Organized as an image-structure
  cloud_plane_xyz->width = cloud_plane_xyz->points.size ();
  cloud_plane_xyz->height = 1;

  /* insert code to set arbitary frame_id setting
   such as frame_id ="/assemble_cloud_1"
  with respect to "/odom or /base_link" */

  // Conversions: PointCloud<T>, PCLPointCloud2, sensor_msgs::PointCloud2
  pcl::PCLPointCloud2 cloud_plane_pcl;
  pcl::toPCLPointCloud2(*cloud_plane_xyz, cloud_plane_pcl);
  sensor_msgs::PointCloud2 cloud_plane_ros;
  pcl_conversions::fromPCL(cloud_plane_pcl, cloud_plane_ros);
  cloud_plane_ros.header.frame_id = "/base_link"; // odom -> /base_link
  cloud_plane_ros.header.stamp = input->header.stamp; // ros::Time::now() -> header.stamp
  pub_plane.publish(cloud_plane_ros);

  /* 3. PCA application to get eigen */
  pcl::PCA<pcl::PointXYZ> pca;
  pca.setInputCloud(cloud_plane_xyz);
  Eigen::Matrix3f eigen_vectors = pca.getEigenVectors(); // 3x3 eigen_vectors(n,m)

  /* 4. PCA Visualization */
  visualization_msgs::Marker points;
  points.header.frame_id = "/base_link"; // odom -> /base_link
  points.header.stamp = input->header.stamp; // ros::Time::now() -> header.stamp
  points.ns = "pca"; // namespace + id
  points.id = 0; // pca/0
  points.action = visualization_msgs::Marker::ADD;
  points.type = visualization_msgs::Marker::ARROW;

  points.pose.orientation.w = 1.0;
  points.scale.x = 0.05;
  points.scale.y = 0.05;
  points.scale.z = 0.05;
  points.color.g = 1.0f;
  points.color.r = 0.0f;
  points.color.b = 0.0f;
  points.color.a = 1.0;

  geometry_msgs::Point p_0, p_1;
  p_0.x = 0; p_0.y = 0; p_0.z = 0; // get from tf
  p_1.x = eigen_vectors(0,0);
  p_1.y = eigen_vectors(0,1); // always negative
  std::cerr << "y = " << eigen_vectors(0,1) << std::endl;
  p_1.z = eigen_vectors(0,2);
  points.points.push_back(p_0);
  points.points.push_back(p_1);
  pub_marker.publish(points);

  /* 5. Point Cloud Rotation  */
  eigen_vectors(0,2) = 0; // ignore very small z-value
  double norm = pow((pow(eigen_vectors(0,0), 2) + pow(eigen_vectors(0,1), 2)), 0.5);
  double nx = eigen_vectors(0,0) / norm;
  double ny = eigen_vectors(0,1) / norm;

  Eigen::Matrix4d rot_z; // rotation inversed, convert to Matrix4f
  rot_z(0,0) = nx; rot_z(0,1) = ny; rot_z(0,2) = 0; rot_z(0,3) = 0; // ny: +/-
  rot_z(1,0) = -ny; rot_z(1,1) = nx; rot_z(1,2) = 0; rot_z(1,3) = 0; // ny: +/-
  rot_z(2,0) = 0; rot_z(2,1) = 0; rot_z(2,2) = 1; rot_z(2,3) = 0;
  rot_z(3,0) = 0; rot_z(3,1) = 0; rot_z(3,2) = 0; rot_z(3,3) = 1;

  // Transformation: Rotation, Translation
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz_rot (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(*cloudPtr, *cloud_xyz); // from PointCloud2 to PointXYZ
  pcl::transformPointCloud(*cloud_xyz, *cloud_xyz_rot, rot_z); // original, transformed, transformation
  pcl::PCLPointCloud2 cloud_rot_pcl;
  sensor_msgs::PointCloud2 cloud_rot_ros;
  pcl::toPCLPointCloud2(*cloud_xyz_rot, cloud_rot_pcl);
  pcl_conversions::fromPCL(cloud_rot_pcl, cloud_rot_ros);
  pub_rot.publish(cloud_rot_ros);

  /* 6. Point Cloud Reduction */
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr >
    vector_cloud_separated_xyz = separate(cloud_xyz_rot, header);
  pcl::PCLPointCloud2 cloud_separated_pcl;
  sensor_msgs::PointCloud2 cloud_separated_ros;

  pcl::toPCLPointCloud2(*vector_cloud_separated_xyz.at(1), cloud_separated_pcl);
  pcl_conversions::fromPCL(cloud_separated_pcl, cloud_separated_ros);
  cloud_separated_ros.header.frame_id = "/base_link"; // odom -> /base_link
  cloud_separated_ros.header.stamp = input->header.stamp; // ros::Time::now() -> header.stamp
  pub_red.publish(cloud_separated_ros);

  // setMarker
  // visualization_msgs::Marker width_min_line;
  // width_min_line.header.frame_id = "/base_link";
  // width_min_line.header.stamp = input->header.stamp; // ros::Time::now() -> header.stamp
  // width_min_line.ns = "width_min";
  // width_min_line.action = visualization_msgs::Marker::ADD;
  // width_min_line.type = visualization_msgs::Marker::LINE_STRIP;
  // width_min_line.pose.orientation.w = 1.0;
  // width_min_line.id = 0;
  // width_min_line.scale.x = 0.025;
  // width_min_line.color.r = 0.0f;
  // width_min_line.color.g = 1.0f;
  // width_min_line.color.b = 0.0f;
  // width_min_line.color.a = 1.0;
  // width_min_line.points.push_back(point_vector.at(0));
  // width_min_line.points.push_back(point_vector.at(2));
  // pub_marker.publish(width_min_line);

  // /* 6. Visualize center line */
  // visualization_msgs::Marker line_strip;
  // line_strip.header.frame_id = "/base_link"; // odom -> /base_link
  // line_strip.header.stamp = input->header.stamp; // ros::Time::now() -> header.stamp
  // line_strip.ns = "center";
  // line_strip.action = visualization_msgs::Marker::ADD;
  // line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  // line_strip.pose.orientation.w = 1.0;
  // line_strip.id = 0; // set id
  // line_strip.scale.x = 0.05;
  // line_strip.color.r = 1.0f;
  // line_strip.color.g = 0.0f;
  // line_strip.color.b = 0.0f;
  // line_strip.color.a = 1.0;
  // // geometry_msgs::Point p_stitch, p_min;
  // p_s.x = 0; p_s.y = 0; p_s.z = 0;
  // p_e.x = p_m.x; p_e.y = p_m.y; p_e.z = 0;
  // line_strip.points.push_back(p_s);
  // line_strip.points.push_back(p_e);
  // pub_marker.publish(line_strip);

  /* PCA Visualization */
  // geometry_msgs::Pose pose; tf::poseEigenToMsg(pca.getEigenVectors, pose);
  /* to use Pose marker in rviz */
  /* Automatic Measurement */
  // 0-a. stitch measurement: -0.5 < z < -0.3
  // 0-b. min width measurement: 0.3 < z < 5m
  // 1. iterate
  // 2. pick point if y < 0
  // 3. compare point with all points if 0 < y
  // 4. pick point-pare recording shortest distance
  // 5. compare the point with previous point
  // 6. update min
  // 7. display value in text in between 2 points
}

int main (int argc, char** argv) {
  // Initialize ROS
  ros::init (argc, argv, "garden_feature_detector");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  // assemble_cloud -> cloud_pcd
  std::cerr << "garden_feature_detector started." << std::endl;
  ros::Subscriber sub = nh.subscribe("assemble_cloud", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub_plane = nh.advertise<sensor_msgs::PointCloud2>("plane", 1);
  pub_voxel = nh.advertise<sensor_msgs::PointCloud2>("voxel", 1);
  pub_rot = nh.advertise<sensor_msgs::PointCloud2>("cloud_rotated", 1);
  pub_polygon_array = nh.advertise<jsk_recognition_msgs::PolygonArray>("polygon_array", 1, 0);
  // pub_rot_plane = nh.advertise<sensor_msgs::PointCloud2>("plane_rotated", 1);
  pub_red = nh.advertise<sensor_msgs::PointCloud2>("cloud_reduced", 1);
  pub_marker = nh.advertise<visualization_msgs::Marker>("marker", 1, 0);

  // Spin
  ros::spin();
}
