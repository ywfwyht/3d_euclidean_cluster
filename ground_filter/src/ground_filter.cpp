#include "ground_filter.h"


GroundFilter::GroundFilter() : node_handle_("~")
{
  ROS_INFO("Inititalizing Ground Filter...");
  node_handle_.param<std::string>("point_topic", point_topic_, "/points_raw");
  ROS_INFO("Input Point Cloud: %s", point_topic_.c_str());
   node_handle_.param("remove_floor",  floor_removal_,  true);
   ROS_INFO("Floor Removal: %d", floor_removal_);
  node_handle_.param("sensor_model", sensor_model_, 64);
  ROS_INFO("Sensor Model: %d", sensor_model_);
  node_handle_.param("sensor_height", sensor_height_, 1.80);
  ROS_INFO("Sensor Height: %f", sensor_height_);
  node_handle_.param("max_slope", max_slope_, 10.0);
  ROS_INFO("Max Slope: %f", max_slope_);
  node_handle_.param("vertical_thres", vertical_thres_, 0.08);
  ROS_INFO("Vertical Threshold: %f", vertical_thres_);

  node_handle_.param<std::string>("no_ground_point_topic", no_ground_topic, "/points_no_ground");
  ROS_INFO("No Ground Output Point Cloud: %s", no_ground_topic.c_str());
  node_handle_.param<std::string>("ground_point_topic", ground_topic, "/points_ground");
  ROS_INFO("Only Ground Output Point Cloud: %s", ground_topic.c_str());


  points_node_sub_ = node_handle_.subscribe(point_topic_, 1000, &GroundFilter::VelodyneCallback, this);
  groundless_points_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>(no_ground_topic, 1000);
  ground_points_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>(ground_topic, 1000);

}
GroundFilter::~GroundFilter() {}

void GroundFilter::FilterGround(const pcl::PointCloud<pcl::PointXYZI>::Ptr &in_cloud,
      pcl::PointCloud<pcl::PointXYZI> &out_groundless_points,
      pcl::PointCloud<pcl::PointXYZI> &out_ground_points)

{
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr result_cloud1(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr result_cloud2(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PassThrough<pcl::PointXYZI> pass;
  pass.setInputCloud(in_cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(-0.5, 0.4);
  pass.setFilterLimitsNegative(false); //ground points
  pass.filter(*filtered_cloud);

  pass.setFilterLimitsNegative(true); //no ground points
  pass.filter(*result_cloud1);
  // std::cerr << "==============  " << filtered_cloud->size() << std::endl;
  // std::cerr << "--------------  " << result_cloud1->size() << std::endl;

  // sensor_msgs::PointCloud2::Ptr output_cloud(new sensor_msgs::PointCloud2);
  // pcl::toROSMsg(*filtered_cloud, *output_cloud);
  // ground_points_pub_.publish(*output_cloud);

  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr idx(new pcl::PointIndices);

  pcl::SACSegmentation<pcl::PointXYZI> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.15);
  seg.setInputCloud(filtered_cloud);
  seg.segment(*idx, *coefficients);

  pcl::ExtractIndices<pcl::PointXYZI> extract;
  extract.setInputCloud(filtered_cloud);
  extract.setIndices(idx);
  extract.setNegative(false);
  extract.filter(out_ground_points);

  extract.setNegative(true);
  extract.filter(*result_cloud2);
  out_groundless_points = *result_cloud1 + *result_cloud2;

  // std::cerr << "--------------  " << out_ground_points.size() << std::endl;
  // sensor_msgs::PointCloud2 ground_cloud, no_ground_cloud;
  // pcl::toROSMsg(out_ground_points, ground_cloud);
  // pcl::toROSMsg(out_groundless_points, no_ground_cloud);
  // ground_points_pub_.publish(ground_cloud);
  // groundless_points_pub_.publish(no_ground_cloud);
  // #pragma omp for
  //   for (int i = 0; i < point_cloud_ptr->points.size(); i++)
  //   {
  //       if (point_cloud_ptr->points[i].z >= -1.16 && point_cloud_ptr->points[i].z <= -0.96)
  //       {
  //           GroundinitPoint_ptr->points.push_back(point_cloud_ptr->points[i]);
  //       }
  //   }

}

void GroundFilter::VelodyneCallback(const sensor_msgs::PointCloud2::ConstPtr &in_cloud_msg)
{

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI> vertical_points;
  pcl::PointCloud<pcl::PointXYZI> ground_points;

  pcl::fromROSMsg(*in_cloud_msg, *cloud);
  vertical_points.header = cloud->header;
  ground_points.header = cloud->header;
  vertical_points.clear();
  ground_points.clear();

  FilterGround(cloud, vertical_points, ground_points);

  if (!floor_removal_)
  {
    vertical_points = *cloud;
  }

  // groundless_points_pub_.publish(vertical_points);
  // ground_points_pub_.publish(ground_points);
  sensor_msgs::PointCloud2 ground_cloud, no_ground_cloud;
  pcl::toROSMsg(ground_points, ground_cloud);
  pcl::toROSMsg(vertical_points, no_ground_cloud);
  ground_points_pub_.publish(ground_cloud);
  groundless_points_pub_.publish(no_ground_cloud);
  ROS_INFO("points to publish successfully !");
  // std::cerr << " points to publish successfully !" << std::endl;
  
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "ground_filter");
  GroundFilter node;
  ros::spin();

  return 0;

}
