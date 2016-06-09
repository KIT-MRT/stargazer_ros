#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "ImageUndistorterInterface.h"

namespace stargazer_ros_tool {

class ImageUndistorterInterfaceNodelet : public nodelet::Nodelet {

  virtual void onInit();
  std::unique_ptr<ImageUndistorterInterface> m_;
};

void ImageUndistorterInterfaceNodelet::onInit() {
  m_ = std::make_unique<ImageUndistorterInterface>(getNodeHandle(),
                                                   getPrivateNodeHandle());
}
} // namespace image_undistorter

PLUGINLIB_DECLARE_CLASS(stargazer_ros_tool, ImageUndistorterInterfaceNodelet,
                        stargazer_ros_tool::ImageUndistorterInterfaceNodelet,
                        nodelet::Nodelet);
