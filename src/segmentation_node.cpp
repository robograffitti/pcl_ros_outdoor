#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
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

ros::Publisher pub_v; // voxel publisher
ros::Publisher pub_p;
ros::Publisher pub_f;
// How to avoid hard-coding a topic name?
ros::Publisher pub_c; // publish cluster

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input) {
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
  seg.setDistanceThreshold (0.02);

  /* Filtering Object: Extraction */
  // pcl::ExtractIndices<pcl::PointXYZ> extract;

  int i = 0, nr_points = (int) cloud_templated->points.size ();
  // While 30% of the original cloud is still there
  while (cloud_templated->points.size () > 0.3 * nr_points)
    {
      // Segment the largest planar component from the remaining cloud
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

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_templated);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.02); // 0.02 = 2ch
  ec.setMinClusterSize (100); // parameter
  ec.setMaxClusterSize (25000); // parameter
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_templated);
  ec.extract (cluster_indices);

  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin ();
       it != cluster_indices.end (); ++it) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
        cloud_cluster->points.push_back (cloud_templated->points[*pit]); //*
      cloud_cluster->width = cloud_cluster->points.size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;

      // Convert data type and publish, but not sure which cluster is published!
      pcl::PCLPointCloud2 out_c;
      sensor_msgs::PointCloud2 output_c;
      pcl::toPCLPointCloud2(*cloud_cluster, out_c);
      pcl_conversions::fromPCL(out_c, output_c);
      pub_p.publish(output_c);
      j++;
    }

  // Convert to ROS data type
  pcl::PCLPointCloud2 out_p;
  pcl::PCLPointCloud2 out_f;
  sensor_msgs::PointCloud2 output_p;
  sensor_msgs::PointCloud2 output_f;
  pcl::toPCLPointCloud2(*cloud_p, out_p);
  pcl::toPCLPointCloud2(*cloud_f, out_f);
  pcl_conversions::fromPCL(out_p, output_p);
  pcl_conversions::fromPCL(out_f, output_f);
  pub_p.publish(output_p);
  pub_f.publish(output_f); 
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
  ros::Subscriber sub = nh.subscribe("assemble_cloud", 1, cloud_cb);
  
  // Create a ROS publisher for the output point cloud
  // A node has both of publisheres and subscribers.
  pub_p = nh.advertise<sensor_msgs::PointCloud2>("output_p", 1);
  pub_f = nh.advertise<sensor_msgs::PointCloud2>("output_f", 1);
  pub_v = nh.advertise<sensor_msgs::PointCloud2>("output_v", 1);
  pub_c = nh.advertise<sensor_msgs::PointCloud2>("cluster_c", 1);
  // pub = nh.advertise<pcl_msgs::ModelCoefficients>("output", 1);
  // pub = nh.advertise<pcl_msgs::PointIndices>("output", 1);
  // Spin
  ros::spin();
}
