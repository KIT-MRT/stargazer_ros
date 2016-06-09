#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "LandmarkFinderInterface.h"

namespace stargazer_ros_tool {

class LandmarkFinderInterfaceNodelet : public nodelet::Nodelet {

  virtual void onInit();
  std::unique_ptr<LandmarkFinderInterface> m_;
};

void LandmarkFinderInterfaceNodelet::onInit() {
  m_ = std::make_unique<LandmarkFinderInterface>(getNodeHandle(),
                                                 getPrivateNodeHandle());
}
} // namespace image_undistorter

PLUGINLIB_DECLARE_CLASS(stargazer_ros_tool, LandmarkFinderInterfaceNodelet,
                        stargazer_ros_tool::LandmarkFinderInterfaceNodelet,
                        nodelet::Nodelet);
