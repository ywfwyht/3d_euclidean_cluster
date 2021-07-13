

#ifndef __GROUND_FILTER_H__
#define __GROUND_FILTER_H__

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <opencv/cv.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>


class GroundFilter
{
public:

  GroundFilter();
  ~GroundFilter();

private:

  ros::NodeHandle node_handle_;
  ros::Subscriber points_node_sub_;
  ros::Publisher groundless_points_pub_;
  ros::Publisher ground_points_pub_;

  std::string point_topic_;
  std::string no_ground_topic, ground_topic;
  int     sensor_model_;
  double     sensor_height_;
  double     max_slope_;
  double vertical_thres_;
  bool    floor_removal_;


  void VelodyneCallback(const sensor_msgs::PointCloud2::ConstPtr &in_cloud_msg);
  void FilterGround(const pcl::PointCloud<pcl::PointXYZI>::Ptr &in_cloud,
                          pcl::PointCloud<pcl::PointXYZI> &out_groundless_points,
                          pcl::PointCloud<pcl::PointXYZI> &out_ground_points);

};


#endif