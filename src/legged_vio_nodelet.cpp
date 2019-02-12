
#include <legged_vio/legged_vio_nodelet.h>

namespace legged_vio {
void LeggedVioNodelet::onInit() {
  legged_vio_ptr.reset(new LeggedVio(getPrivateNodeHandle()));
  if (!legged_vio_ptr->initialize()) {
    ROS_ERROR("Cannot initialize Legged VIO...");
    return;
  }
  return;
}

//PLUGINLIB_DECLARE_CLASS(legged_vio, leggedVioNodelet,legged_vio::leggedVioNodelet, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(legged_vio::LeggedVioNodelet, nodelet::Nodelet);

} // end namespace legged_vio
