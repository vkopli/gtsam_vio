
#ifndef LEGGED_VIO_NODELET_H
#define LEGGED_VIO_NODELET_H

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <legged_vio/legged_vio.h>

namespace legged_vio {
class LeggedVioNodelet : public nodelet::Nodelet {
public:
  LeggedVioNodelet() { return; }
  ~LeggedVioNodelet() { return; }

private:
  virtual void onInit();
  LeggedVioPtr legged_vio_ptr;
};
} // end namespace legged_vio

#endif

