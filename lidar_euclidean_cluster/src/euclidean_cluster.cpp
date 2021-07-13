#include "euclidean_cluster.h"



EuClusterCore::EuClusterCore(ros::NodeHandle &nh)
{
    seg_distance_ = {15, 30, 45, 60};
    cluster_distance_ = {0.5, 1.0, 1.5, 2.0, 2.5};
    sub_point_cloud_ = nh.subscribe("/points_no_ground", 5, &EuClusterCore::point_callback, this);

    pub_bounding_boxs_ = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("/detected_bounding_boxs", 5);

    ros::spin();
}

EuClusterCore::~EuClusterCore() {}

void EuClusterCore::publish_cloud(const ros::Publisher &in_publisher,
                                  const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_to_publish_ptr,
                                  const std_msgs::Header &in_header)
{
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
    cloud_msg.header = in_header;
    // std::cerr << "header: " << in_header << std::endl;
    in_publisher.publish(cloud_msg);
}

void EuClusterCore::voxel_grid_filer(pcl::PointCloud<pcl::PointXYZ>::Ptr in, pcl::PointCloud<pcl::PointXYZ>::Ptr out, double leaf_size)
{
#pragma omp for
    pcl::PointCloud<pcl::PointXYZ>::Ptr filter_in(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_x(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_y(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_z(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass_x;
    pcl::PassThrough<pcl::PointXYZ> pass_y;
    pcl::PassThrough<pcl::PointXYZ> pass_z;
    // for(int i = 0; i < in->points.size(); i++)
    // {
    //     if(sqrt(pow(in->points[i].x + 1, 2) + pow(in->points[i].y + 1, 2)) > 2)
    //     {
    //         filter_in->points.push_back(in->points[i]);

    //     }

    // }
    pass_x.setInputCloud(in);
	pass_x.setFilterFieldName("x");
	pass_x.setFilterLimits(-20, 60);
    // pass.setNegative(true);
	pass_x.filter(*filtered_x);

    pass_y.setInputCloud(filtered_x);
	pass_y.setFilterFieldName("y");
	pass_y.setFilterLimits(-15, 15);
    // pass.setNegative(true);
	pass_y.filter(*filtered_y);

    pass_z.setInputCloud(filtered_y);
	pass_z.setFilterFieldName("z");
	pass_z.setFilterLimits(-1, 3);
    // pass.setNegative(true);
	pass_z.filter(*filtered_z);
    // pass_z.filter(*out);

    

    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setInputCloud(filtered_z);
    filter.setLeafSize(leaf_size, leaf_size, leaf_size);
    filter.filter(*out);
    
    sensor_msgs::PointCloud2 filtered_cloud;
    // ros::Publisher filtered_pub;
    // std_msgs::Header point_header;

    pcl::toROSMsg(*out, filtered_cloud);
    filtered_cloud.header = point_cloud_header_;
    pub_filtered_cloud = filtered_nh.advertise<sensor_msgs::PointCloud2>("/voxel_filtered_cloud", 100, true);
    pub_filtered_cloud.publish(filtered_cloud);
    // std::cerr<< "1111:" << point_cloud_header_ << std::endl;

}

void EuClusterCore::cluster_segment(pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc,
                                    double in_max_cluster_distance, std::vector<Detected_Obj> &obj_list)
{

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

    //create 2d pc
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2d(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*in_pc, *cloud_2d);
    //make it flat
    for (size_t i = 0; i < cloud_2d->points.size(); i++)
    {
        cloud_2d->points[i].z = 0;
    }

    if (cloud_2d->points.size() > 0)
        tree->setInputCloud(cloud_2d);

    std::vector<pcl::PointIndices> local_indices;

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> euclid;
    euclid.setInputCloud(cloud_2d);
    euclid.setClusterTolerance(in_max_cluster_distance);
    euclid.setMinClusterSize(MIN_CLUSTER_SIZE);
    euclid.setMaxClusterSize(MAX_CLUSTER_SIZE);
    euclid.setSearchMethod(tree);
    euclid.extract(local_indices);

    int block_num = 0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_cluster(new pcl::PointCloud<pcl::PointXYZ>);

    for (size_t i = 0; i < local_indices.size(); i++)
    {
        // the structure to save one detected object
        Detected_Obj obj_info;

        float min_x = std::numeric_limits<float>::max();
        float max_x = -std::numeric_limits<float>::max();
        float min_y = std::numeric_limits<float>::max();
        float max_y = -std::numeric_limits<float>::max();
        float min_z = std::numeric_limits<float>::max();
        float max_z = -std::numeric_limits<float>::max();

        for (auto pit = local_indices[i].indices.begin(); pit != local_indices[i].indices.end(); ++pit)
        {
            //fill new colored cluster point by point
            pcl::PointXYZ p;
            p.x = in_pc->points[*pit].x;
            p.y = in_pc->points[*pit].y;
            p.z = in_pc->points[*pit].z;

            obj_info.centroid_.x += p.x;
            obj_info.centroid_.y += p.y;
            obj_info.centroid_.z += p.z;

            current_cluster->points.push_back(p);

            if (p.x < min_x)
                min_x = p.x;
            if (p.y < min_y)
                min_y = p.y;
            if (p.z < min_z)
                min_z = p.z;
            if (p.x > max_x)
                max_x = p.x;
            if (p.y > max_y)
                max_y = p.y;
            if (p.z > max_z)
                max_z = p.z;
        }

        //min, max points
        obj_info.min_point_.x = min_x;
        obj_info.min_point_.y = min_y;
        obj_info.min_point_.z = min_z;

        obj_info.max_point_.x = max_x;
        obj_info.max_point_.y = max_y;
        obj_info.max_point_.z = max_z;

        //calculate centroid, average
        if (local_indices[i].indices.size() > 0)
        {
            obj_info.centroid_.x /= local_indices[i].indices.size();
            obj_info.centroid_.y /= local_indices[i].indices.size();
            obj_info.centroid_.z /= local_indices[i].indices.size();
        }

        //calculate bounding box
        double length_ = obj_info.max_point_.x - obj_info.min_point_.x;
        double width_ = obj_info.max_point_.y - obj_info.min_point_.y;
        double height_ = obj_info.max_point_.z - obj_info.min_point_.z;

        obj_info.bounding_box_.header = point_cloud_header_;

        obj_info.bounding_box_.pose.position.x = obj_info.min_point_.x + length_ / 2;
        obj_info.bounding_box_.pose.position.y = obj_info.min_point_.y + width_ / 2;
        obj_info.bounding_box_.pose.position.z = obj_info.min_point_.z + height_ / 2;

        obj_info.bounding_box_.dimensions.x = ((length_ < 0) ? -1 * length_ : length_);
        obj_info.bounding_box_.dimensions.y = ((width_ < 0) ? -1 * width_ : width_);
        obj_info.bounding_box_.dimensions.z = ((height_ < 0) ? -1 * height_ : height_);

        // // pose estimation
        // double rz = 0;

        // {
        //     std::vector<cv::Point2f> points;
        //     for (unsigned int i = 0; i < current_cluster->points.size(); i++)
        //     {
        //     cv::Point2f pt;
        //     pt.x = current_cluster->points[i].x;
        //     pt.y = current_cluster->points[i].y;
        //     points.push_back(pt);
        //     }

        //     std::vector<cv::Point2f> hull;
        //     cv::convexHull(points, hull);

        //     polygon_.header = point_cloud_header_;
        //     for (size_t i = 0; i < hull.size() + 1; i++)
        //     {
        //     geometry_msgs::Point32 point;
        //     point.x = hull[i % hull.size()].x;
        //     point.y = hull[i % hull.size()].y;
        //     point.z = min_z;
        //     polygon_.polygon.points.push_back(point);
        //     }

        //     if (true)
        //     {
        //         cv::RotatedRect box = minAreaRect(hull);
        //         rz = box.angle * 3.14 / 180;
        //         obj_info.bounding_box_.pose.position.x = box.center.x;
        //         obj_info.bounding_box_.pose.position.y = box.center.y;
        //         obj_info.bounding_box_.dimensions.x = box.size.width;
        //         obj_info.bounding_box_.dimensions.y = box.size.height;
        //     }
        // }

        // // set bounding box direction
        // tf::Quaternion quat = tf::createQuaternionFromRPY(0.0, 0.0, rz);
        // tf::quaternionTFToMsg(quat, obj_info.bounding_box_.pose.orientation);

        obj_list.push_back(obj_info);
    }
}

void EuClusterCore::cluster_by_distance(pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc, std::vector<Detected_Obj> &obj_list)
{
    //cluster the pointcloud according to the distance of the points using different thresholds (not only one for the entire pc)
    //in this way, the points farther in the pc will also be clustered

    //0 => 0-15m d=0.5
    //1 => 15-30 d=1
    //2 => 30-45 d=1.5
    //3 => 45-60 d=2.0
    //4 => >60   d=2.5

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> segment_pc_array(5);

    for (size_t i = 0; i < segment_pc_array.size(); i++)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
        segment_pc_array[i] = tmp;
    }

    for (size_t i = 0; i < in_pc->points.size(); i++)
    {
        pcl::PointXYZ current_point;
        current_point.x = in_pc->points[i].x;
        current_point.y = in_pc->points[i].y;
        current_point.z = in_pc->points[i].z;

        float origin_distance = sqrt(pow(current_point.x, 2) + pow(current_point.y, 2));

        // 如果点的距离大于45m, 忽略该点
        if (origin_distance >= 100)
        // if (origin_distance >= 45)
        {
            continue;
        }

        if (origin_distance < seg_distance_[0])
        {
            segment_pc_array[0]->points.push_back(current_point);
        }
        else if (origin_distance < seg_distance_[1])
        {
            segment_pc_array[1]->points.push_back(current_point);
        }
        else if (origin_distance < seg_distance_[2])
        {
            segment_pc_array[2]->points.push_back(current_point);
        }
        else if (origin_distance < seg_distance_[3])
        {
            segment_pc_array[3]->points.push_back(current_point);
        }
        else
        {
            segment_pc_array[4]->points.push_back(current_point);
        }
    }

    std::vector<pcl::PointIndices> final_indices;
    std::vector<pcl::PointIndices> tmp_indices;

    for (size_t i = 0; i < segment_pc_array.size(); i++)
    {
        cluster_segment(segment_pc_array[i], cluster_distance_[i], obj_list);
    }
}

void EuClusterCore::point_callback(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    point_cloud_header_ = in_cloud_ptr->header;
    std::cerr << "cloud header: " << in_cloud_ptr->header << std::endl;

    pcl::fromROSMsg(*in_cloud_ptr, *current_pc_ptr);
    // down sampling the point cloud before cluster
    voxel_grid_filer(current_pc_ptr, filtered_pc_ptr, LEAF_SIZE);

    std::vector<Detected_Obj> global_obj_list;
    cluster_by_distance(filtered_pc_ptr, global_obj_list);

    jsk_recognition_msgs::BoundingBoxArray bbox_array;

    for (size_t i = 0; i < global_obj_list.size(); i++)
    {
        bbox_array.boxes.push_back(global_obj_list[i].bounding_box_);
    }
    bbox_array.header = point_cloud_header_;

    pub_bounding_boxs_.publish(bbox_array);
}
