#include "simple_voxelizer/simple_voxelizer.h"

#include <pcl_ros/transforms.h>
#include <sensor_msgs/point_cloud2_iterator.h>

namespace simple_voxelizer {

SimpleVoxelizer::SimpleVoxelizer(ros::NodeHandle& nh) : nh_(nh), tf_buffer_(), tf2_listener_(tf_buffer_) {
    nh.getParam("env_center", env_center_);
    ROS_ASSERT((env_center_.size() == 3) && "env_center parameter must be a 3-value array!");
    nh.getParam("env_range", env_range_);
    ROS_ASSERT((env_range_.size() == 3) && "env_range parameter must be a 3-value array!");
    if (!nh_.getParam("env_resolution", env_resolution_)) {
        ROS_FATAL("need a env_resolution paramter!");
    }

    if (!nh_.getParam("world_frame_id", world_frame_id_)) {
        ROS_FATAL("need a world_frame_id paramter!");
    }

    ROS_INFO("using env_center: %f %f %f", env_center_[0], env_center_[1], env_center_[2]);
    ROS_INFO("using env_range:  %f %f %f", env_range_[0], env_range_[1], env_range_[2]);
    ROS_INFO("using env_resolution:  %f", env_resolution_);
    env_origin_.resize(3);
    env_origin_[0] = -env_range_[0] + env_resolution_ / 2 + env_center_[0];
    env_origin_[1] = -env_range_[1] + env_resolution_ / 2 + env_center_[1];
    env_origin_[2] = -env_range_[2] + env_resolution_ / 2 + env_center_[2];

    num_voxels_.resize(3);
    num_voxels_[0] = std::round(env_range_[0] * 2 / env_resolution_);
    num_voxels_[1] = std::round(env_range_[1] * 2 / env_resolution_);
    num_voxels_[2] = std::round(env_range_[2] * 2 / env_resolution_);
    ROS_INFO("num_voxels:  %d %d %d", num_voxels_[0], num_voxels_[1], num_voxels_[2]);
    bin_occ_msg_.occupancies.resize(num_voxels_[0] * num_voxels_[1] * num_voxels_[2]);

    // fitler replace value
    // nh_.param<double> ("filter_replace_value", filter_replace_value_, 0);
    // ROS_INFO ("using filter replace value %f", filter_replace_value_);

    bin_occ_pub_ = nh_.advertise<simple_voxelizer_msgs::BinaryOccupancies>("output", 3);
    transformed_point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("transformed_point_cloud", 3);
    depth_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("input", 3, &SimpleVoxelizer::callback, this);
}

SimpleVoxelizer::~SimpleVoxelizer() {}

// helper function to get current time
// double SimpleVoxelizer::getTime ()
// {
//   timeval current_time;
//   gettimeofday (&current_time, NULL);
//   return (current_time.tv_sec + 1e-6 * current_time.tv_usec);
// }

void SimpleVoxelizer::callback(const sensor_msgs::PointCloud2::ConstPtr& point_cloud) {
    // if (!tf_initalized_) {
    //     ROS_INFO("waiting for tf initialization... (assuming fixed, static sensor)");
    //     tf::TransformListener listener;
    //     listener.waitForTransform(world_frame_id_, point_cloud->header.frame_id, ros::Time(0));
    //     listener.lookupTransform(world_frame_id_, point_cloud->header.frame_id, ros::Time(0), sensor_transform_);
    //     ROS_INFO("done.");
    // }
    if (bin_occ_pub_.getNumSubscribers() > 0) {
        sensor_msgs::PointCloud2::Ptr pc_transformed(new sensor_msgs::PointCloud2);
        pcl_ros::transformPointCloud(world_frame_id_, *point_cloud, *pc_transformed, tf_buffer_);
        //pcl_ros::transformPointCloud(world_frame_id, sensor_transform_, pc2, pc2_transformed);
        transformed_point_cloud_pub_.publish(pc_transformed);
        sensor_msgs::PointCloud2Iterator<float> iter_x(*pc_transformed, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*pc_transformed, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*pc_transformed, "z");
        std::fill(bin_occ_msg_.occupancies.begin(), bin_occ_msg_.occupancies.end(), false);
        for (int v = 0; v < int(pc_transformed->height); ++v)
        {
            for (int u = 0; u < int(pc_transformed->width); ++u, ++iter_x, ++iter_y, ++iter_z)
            {
                long ix = std::round((*iter_x - env_origin_[0]) / env_resolution_);
                if (ix < 0 || ix >= num_voxels_[0]) {
                    continue;
                }
                long iy = std::round((*iter_y - env_origin_[1]) / env_resolution_);
                if (iy < 0 || iy >= num_voxels_[1]) {
                    continue;
                }
                long iz = std::round((*iter_z - env_origin_[2]) / env_resolution_);
                if (iz < 0 || iz >= num_voxels_[2]) {
                    continue;
                }
                // TODO is this correct? index order: (y, x, z)
                long index = iy * num_voxels_[0] * num_voxels_[2] + ix * num_voxels_[2] + iz;
                bin_occ_msg_.occupancies[index] = true;
            }
        }

        bin_occ_msg_.header = point_cloud->header;
        bin_occ_pub_.publish(bin_occ_msg_);
    }
}

}  // namespace simple_voxelizer
