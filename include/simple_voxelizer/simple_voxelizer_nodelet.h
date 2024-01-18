#pragma once
#include <nodelet/nodelet.h>
#include "simple_voxelizer/simple_voxelizer.h"

namespace simple_voxelizer
{
  class SimpleVoxelizerNodelet : public nodelet::Nodelet
  {
  public:
    SimpleVoxelizerNodelet();
    virtual void onInit();
  private:
    std::shared_ptr<simple_voxelizer::SimpleVoxelizer> voxelizer_;
  };
}

