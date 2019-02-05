/*
 * COPYRIGHT AND PERMISSION NOTICE
 * Penn Software MSCKF_VIO
 * Copyright (C) 2017 The Trustees of the University of Pennsylvania
 * All rights reserved.
 */

#include <legged_vio/image_processor_nodelet.h>

namespace legged_vio {
void ImageProcessorNodelet::onInit() {
  img_processor_ptr.reset(new ImageProcessor(getPrivateNodeHandle()));
  if (!img_processor_ptr->initialize()) {
    ROS_ERROR("Cannot initialize Image Processor...");
    return;
  }
  return;
}

PLUGINLIB_EXPORT_CLASS(legged_vio::ImageProcessorNodelet, nodelet::Nodelet);

} // end namespace legged_vio

