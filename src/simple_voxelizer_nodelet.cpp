#include "simple_voxelizer/simple_voxelizer_nodelet.h"
#include <pluginlib/class_list_macros.h>

namespace simple_voxelizer {
SimpleVoxelizerNodelet::SimpleVoxelizerNodelet() {}

void SimpleVoxelizerNodelet::onInit() {
    NODELET_DEBUG("Initializing nodelet...");

    ros::NodeHandle nh = this->getPrivateNodeHandle();

    voxelizer_.reset(new simple_voxelizer::SimpleVoxelizer(nh));
}
}  // namespace simple_voxelizer

PLUGINLIB_EXPORT_CLASS(simple_voxelizer::SimpleVoxelizerNodelet, nodelet::Nodelet);

