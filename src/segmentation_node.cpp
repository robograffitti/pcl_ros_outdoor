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
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/pca.h>

ros::Publisher pub_v; // voxel publisher
ros::Publisher pub_p;
ros::Publisher pub_f;
ros::Publisher pub_r; // rotated point cloud
ros::Publisher pub_red;
// How to avoid hard-coding a topic name?
ros::Publisher pub_c; // publish cluster
ros::Publisher pub_m; // arrow
ros::Publisher pub_t; // text

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input) {
  std::cerr << "in cloud_cb" << std::endl;
  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered; // (new pcl::PCLPointCloud2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_templated (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
  pcl_conversions::toPCL(*input, *cloud);

  // StatOutlierRemoval
  // How to use both of SOR and VG?
  /* pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> sor_0;
  sor_0.setInputCloud(cloudPtr);
  sor_0.setMeanK(50);
  sor_0.setStddevMulThresh(1.0);
  sor_0.filter(cloud_filtered); // fill in an empty variable */

  // Downsampling
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud(cloudPtr);
  sor.setLeafSize(0.02f, 0.02f, 0.02f);
  sor.filter(cloud_filtered);

  // Publishing voxel
  sensor_msgs::PointCloud2 output_v;
  pcl_conversions::fromPCL(cloud_filtered, output_v);
  pub_v.publish(output_v);

  // Convert
  pcl::fromPCLPointCloud2(cloud_filtered, *cloud_templated); // unused

  /* Perform the actual filtering */
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  seg.setOptimizeCoefficients (true); // Optional
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000); // added
  //seg.setDistanceThreshold (0.02);
  seg.setDistanceThreshold (0.05); //!!!

  /* Segmentation coded by nagahama */
  seg.setInputCloud (cloud_templated);
  seg.segment (*inliers, *coefficients);
  pcl::PCLPointCloud2 out_p;
  //pcl::PointCloud<pcl::PointXYZ> out_c;
  sensor_msgs::PointCloud2 output_p;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
  for (std::vector<int>::const_iterator pit = inliers->indices.begin ();
       pit != inliers->indices.end (); pit++)
    cloud_cluster->points.push_back (cloud_templated->points[*pit]); //*
  cloud_cluster->width = cloud_cluster->points.size ();
  cloud_cluster->height = 1;
  //pcl::toPCLPointCloud2(*inliers, out_p);
  pcl::toPCLPointCloud2(*cloud_cluster, out_p);
  pcl_conversions::fromPCL(out_p, output_p);
  output_p.header.frame_id = "odom";
  output_p.header.stamp = ros::Time::now();
  pub_p.publish(output_p); // 
  /* Segmentation coded by nagahama */

  /* PCA coded by nagahama */
  pcl::PCA<pcl::PointXYZ> pca;
  pca.setInputCloud(cloud_cluster);
  Eigen::Matrix3f eigen_vectors = pca.getEigenVectors();//<<
  // std::cerr << eigen_vectors(0,0) << ", " << eigen_vectors(0,1) << ", " << eigen_vectors(0,2) << std::endl;
  //std::cerr << eigen_vectors(1,0) << ", " << eigen_vectors(1,1) << ", " << eigen_vectors(1,2) << std::endl;
  /* PCA coded by nagahama */

  /* Point Cloud Rotation  */
  eigen_vectors(0,2) = 0;
  double norm = pow((eigen_vectors(0,0) * eigen_vectors(0,0) + eigen_vectors(0,1) * eigen_vectors(0,1)), 0.5);
  double nx = eigen_vectors(0,0) / norm;
  double ny = eigen_vectors(0,1) / norm;

  Eigen::Matrix4d rot_z; // = new Eigen::Matrix3d; // rotation inversed!
  rot_z(0,0) = nx; rot_z(0,1) = ny; rot_z(0,2) = 0; rot_z(0,3) = 0; 
  rot_z(1,0) = -ny; rot_z(1,1) = nx; rot_z(1,2) = 0; rot_z(1,3) = 0; 
  rot_z(2,0) = 0; rot_z(2,1) = 0; rot_z(2,2) = 1; rot_z(2,3) = 0; 
  rot_z(3,0) = 0; rot_z(3,1) = 0; rot_z(3,2) = 0; rot_z(3,3) = 1;
  // Error output
  std::cerr << rot_z(0,0) << ", " << rot_z(0,1) << ", " << rot_z(0,2) << ", " << rot_z(0,3) << std::endl;
  std::cerr << rot_z(1,0) << ", " << rot_z(1,1) << ", " << rot_z(1,2) << ", " << rot_z(1,3) << std::endl;
  std::cerr << rot_z(2,0) << ", " << rot_z(2,1) << ", " << rot_z(2,2) << ", " << rot_z(2,3) << std::endl;
  std::cerr << rot_z(3,0) << ", " << rot_z(3,1) << ", " << rot_z(3,2) << ", " << rot_z(3,3) << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz_rot (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(*cloudPtr, *cloud_xyz); // conversion

  // Transform
  pcl::transformPointCloud(*cloud_xyz, *cloud_xyz_rot, rot_z);

  // conversion for cloud_rotated
  pcl::PCLPointCloud2 cloud_rotated;
  sensor_msgs::PointCloud2 output_r;
  pcl::toPCLPointCloud2(*cloud_xyz_rot, cloud_rotated);
  pcl_conversions::fromPCL(cloud_rotated, output_r);
  pub_r.publish(output_r);
  /* Point Cloud Rotation */

  /* Reduce range of cloud_xyz_rot */
  std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> > cloud_xyz_rot_vector;
  cloud_xyz_rot_vector = cloud_xyz_rot->points;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_reduced_xyz (new pcl::PointCloud<pcl::PointXYZ>);

  for (std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> >::const_iterator itr =
         cloud_xyz_rot_vector.begin(); itr != cloud_xyz_rot_vector.end(); ++itr) {
    // pcl::PointXYZ has members, x, y, and z
    if (1.50 < itr->x && itr->x < 1.675) { // 1.5~1.75 or 1.75~2.00
      if (-0.675 < itr->y && itr->y < 0.675) {
        if (-0.3125 < itr->z && itr->z < 5.00) {
          pcl::PointXYZ p;
          p.x = itr->x; p.y = itr->y; p.z = itr->z;
          cloud_reduced_xyz->points.push_back(p);
        }
      }
    }
  }

  // Conversion for visualization
  pcl::PCLPointCloud2 cloud_reduced;
  pcl::toPCLPointCloud2(*cloud_reduced_xyz, cloud_reduced);
  sensor_msgs::PointCloud2 cloud_red;
  pcl_conversions::fromPCL(cloud_reduced, cloud_red);
  cloud_red.header.frame_id = "odom";
  cloud_red.header.stamp = ros::Time::now();
  pub_red.publish(cloud_red);

  /* Reduce range of cloud_xyz_rot */

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

  double width_min = 2.0; // initialize with a constant
  double width_stitch = 4.0;
  pcl::PointXYZ p_s, p_e, p_m;
  for (std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> >::const_iterator itr_1 =
         cloud_reduced_xyz->points.begin(); itr_1 != cloud_reduced_xyz->points.end(); ++itr_1) {
    if (itr_1->y < 0) {
      for (std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> >::const_iterator itr_2 =
             cloud_reduced_xyz->points.begin(); itr_2 != cloud_reduced_xyz->points.end(); ++itr_2) {
        if (0 <= itr_2->y) {
          double tmp;
          tmp = sqrt(pow(fabs(itr_1->x - itr_2->x), 2)
                     + pow(fabs(itr_1->y - itr_2->y), 2)
                     + pow(fabs(itr_1->z - itr_2->z), 2));
          if (tmp <= width_min) {
            width_min = tmp;
            p_m.x = itr_1->x;
            p_m.y = itr_1->y + sqrt(pow(fabs(itr_1->y - itr_2->y), 2)) / 2;
            p_m.z = itr_1->z;
            p_s.x = itr_1->x; p_s.y = itr_1->y; p_s.z = itr_1->z;
            p_e.x = itr_2->x; p_e.y = itr_2->y; p_e.z = itr_2->z;
          } else if (-0.3125 < itr_1->z && itr_1->z < 0.325 && tmp <= width_stitch) {
            width_stitch = tmp;
          }
        }
      }
    }
  }
  std::cerr << "width_min = " << width_min << std::endl
            << "width_stitch = " << width_stitch << std::endl
            <<", point inbetween = "  << std::endl
            << "(" << p_s.x << ", " << p_s.y << ", " << p_s.z << ")" << std::endl
            << "(" << p_e.x << ", " << p_e.y << ", " << p_e.z << ")" << std::endl
            << "(" << p_m.x << ", " << p_m.y << ", " << p_m.z << ")" << std::endl;

  // Display the value
  visualization_msgs::Marker texts; // TEXT_VIEW_FACING
  texts.header.frame_id = "/odom";
  texts.header.stamp = ros::Time::now();
  texts.ns = "texts";
  texts.action = visualization_msgs::Marker::ADD;
  texts.id = 1;
  texts.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  texts.pose.position.x = p_m.x;
  texts.pose.position.y = p_m.y;
  texts.pose.position.z = -0.500;
  texts.pose.orientation.x = 0.0;
  texts.pose.orientation.y = 0.0;
  texts.pose.orientation.z = 0.0;
  texts.pose.orientation.w = 1.0;
  texts.scale.x = 0.2;
  texts.scale.y = 0.2;
  texts.scale.z = 0.2;
  texts.color.r = 0.0f;
  texts.color.g = 1.0f;
  texts.color.b = 0.0f;
  texts.color.a = 1.0;
  std::ostringstream strs; strs << width_stitch;
  std::string str = strs.str();
  texts.text = str;
  pub_t.publish(texts);

  // geometry_msgs::Point p_stitch, p_min;
  // p_stitch.x = 0; p_stitch.y = 0; p_stitch.z = 0;

  /* Automatic Measurement */

  /* PCA Visualization */
  // geometry_msgs::Pose pose;
  // tf::poseEigenToMsg(pca.getEigenVectors, pose);
  visualization_msgs::Marker points;
  points.header.frame_id = "/odom";
  points.header.stamp = ros::Time::now();
  points.ns = "points";
  points.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = 1.0;

  points.id = 0;
  points.type = visualization_msgs::Marker::ARROW;
  points.scale.x = 0.1;
  points.scale.y = 0.1;
  points.scale.z = 0.1;
  // Points are green
  points.color.g = 1.0f;
  points.color.r = 0.0f;
  points.color.b = 0.0f;
  points.color.a = 1.0;

  geometry_msgs::Point p_0, p_1;
  // p_0.x = 0;
  // p_0.y = 0;
  // p_0.z = 0;
  // p_1.x = eigen_vectors(0,0);
  // p_1.y = eigen_vectors(0,1);
  // p_1.z = eigen_vectors(0,2);

  p_0.x = p_s.x;
  p_0.y = p_s.y;
  p_0.z = p_s.z;
  p_1.x = p_e.x;
  p_1.y = p_e.y;
  p_1.z = p_e.z;

  points.points.push_back(p_0);
  points.points.push_back(p_1);

  pub_m.publish(points);
  /* PCA Visualizaton */

  /* Filtering Object: Extraction */
  // pcl::ExtractIndices<pcl::PointXYZ> extract;
#ifdef DEBUG__
  int i = 0, nr_points = (int) cloud_templated->points.size ();
  // While 30% of the original cloud is still there
  while (cloud_templated->points.size () > 0.3 * nr_points)
    {      // Segment the largest planar component from the remaining cloud
      seg.setInputCloud (cloud_templated);
      seg.segment (*inliers, *coefficients);
      if (inliers->indices.size () == 0)
        {
          ROS_INFO("Could not estimate a planar model for the given dataset.");
          break;
        }

      // Extract the inliers
      pcl::ExtractIndices<pcl::PointXYZ> extract;
      extract.setInputCloud (cloud_templated);
      extract.setIndices (inliers);
      extract.setNegative (false);

      // Get the points associated with the planar surface
      extract.filter (*cloud_p);

      // Remove the planar inliers, extract the rest
      extract.setNegative (true);
      extract.filter (*cloud_f);
      *cloud_templated = *cloud_f;
      // cloud_templated.swap (cloud_f);
      i++;
    }
#endif

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_templated);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(0.05); // 0.02 = 2ch //(0.02); // 0.02 = 2ch
  ec.setMinClusterSize (500); // parameter
  ec.setMaxClusterSize (25000); // parameter//(25000); // parameter
  ec.setSearchMethod (tree);
  //ec.setInputCloud (cloud_templated);
  ec.setInputCloud (cloud_templated);
  ec.extract (cluster_indices);

  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin ();
       it != cluster_indices.end (); ++it) {
    std::cerr << j << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
      cloud_cluster->points.push_back (cloud_templated->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    // Convert data type and publish, but not sure which cluster is published!
    pcl::PCLPointCloud2 out_c;
    sensor_msgs::PointCloud2 output_c;
    // pcl::toPCLPointCloud2(*cloud_cluster, out_c);
    // //pcl::toPCLPointCloud2(*cloud_templated, out_c);//temp
    // pcl_conversions::fromPCL(out_c, output_c);
    // output_c.header.frame_id = "odom";
    // output_c.header.stamp = ros::Time::now();
    // pub_p.publish(output_c);
    j++;
    break;
  }


  // Convert to ROS data type
  // pcl::PCLPointCloud2 out_p;
  // pcl::PCLPointCloud2 out_f;
  // sensor_msgs::PointCloud2 output_p;
  // sensor_msgs::PointCloud2 output_f;
  // pcl::toPCLPointCloud2(*cloud_p, out_p);
  // pcl::toPCLPointCloud2(*cloud_f, out_f);
  // pcl_conversions::fromPCL(out_p, output_p);
  // pcl_conversions::fromPCL(out_f, output_f);
  //pub_p.publish(output_p);
  // pub_f.publish(output_f); 
  // output = *input; // only for debuging
  // ROS_INFO_STREAM("cloud_width:" << output.width);
  // pcl_conversions::fromPCL(cloud_filtered, output);

  // Publish the model coefficients
  // pcl_msgs::ModelCoefficients ros_coefficients;
  // pcl_msgs::PointIndices ros_inliers;
  // pcl_conversions::fromPCL(coefficients, ros_coefficients);
  // pcl_conversions::fromPCL(inliers, ros_inliers);
  // pub.publish(ros_coefficients);
  // pub.publish(ros_inliers);
}

int main (int argc, char** argv) {
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;
  
  // Create a ROS subscriber for the input point cloud
  // when topic /input is incoming, cloud_cb callback is called
  // /assemble_cloud -> /cloud_pcd
  std::cerr << "my_pcl_tutorial started." << std::endl;
  ros::Subscriber sub = nh.subscribe("cloud_pcd", 1, cloud_cb);
  
  // Create a ROS publisher for the output point cloud
  // A node has both of publisheres and subscribers.
  pub_p = nh.advertise<sensor_msgs::PointCloud2>("output_p", 1);
  // pub_f = nh.advertise<sensor_msgs::PointCloud2>("output_f", 1);
  pub_v = nh.advertise<sensor_msgs::PointCloud2>("output_v", 1);
  pub_r = nh.advertise<sensor_msgs::PointCloud2>("cloud_rot", 1);
  pub_red = nh.advertise<sensor_msgs::PointCloud2>("cloud_red", 1);
  // pub_c = nh.advertise<sensor_msgs::PointCloud2>("cluster_c", 1);
  // pub = nh.advertise<pcl_msgs::ModelCoefficients>("output", 1);
  // pub = nh.advertise<pcl_msgs::PointIndices>("output", 1);
  pub_m = nh.advertise<visualization_msgs::Marker>("marker", 1, 0);
  pub_t = nh.advertise<visualization_msgs::Marker>("texts", 1, 0);
  // Spin
  ros::spin();
}
