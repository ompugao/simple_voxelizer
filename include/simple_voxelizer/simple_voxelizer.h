#pragma once

#include <ros/node_handle.h>
#include <sensor_msgs/PointCloud2.h>
#include <simple_voxelizer_msgs/BinaryOccupancies.h>
#include <tf2_ros/transform_listener.h>

namespace simple_voxelizer {

class SimpleVoxelizer {
  public:
    SimpleVoxelizer(ros::NodeHandle& nh);

    ~SimpleVoxelizer();

    // callback function that gets ROS images and does everything
    void callback(const sensor_msgs::PointCloud2::ConstPtr& point_cloud);

  protected:
    ros::NodeHandle nh_;
    ros::Subscriber depth_sub_;
    ros::Publisher bin_occ_pub_, transformed_point_cloud_pub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf2_listener_;

    std::string world_frame_id_;
    std::vector<double> env_center_, env_range_, env_origin_;
    std::vector<int> num_voxels_;
    double env_resolution_;

    simple_voxelizer_msgs::BinaryOccupancies bin_occ_msg_;
    // bool tf_initalized_;
    // tf::Transform sensor_transform_;
};

}  // namespace simple_voxelizer
