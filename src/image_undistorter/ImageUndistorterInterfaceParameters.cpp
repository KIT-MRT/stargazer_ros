#include "ImageUndistorterInterfaceParameters.h"

#include <utils_ros/node_handle.hpp>

namespace stargazer_ros_tool {

ImageUndistorterInterfaceParameters& ImageUndistorterInterfaceParameters::getInstance() {
    static ImageUndistorterInterfaceParameters p;
    return p;
}

void ImageUndistorterInterfaceParameters::fromNodeHandle(const ros::NodeHandle& node_handle) {
    using namespace utils_ros;

    getParam(node_handle, "calib_file", calib_file);
    getParam(node_handle, "raw_image_topic", raw_image_topic);
    getParam(node_handle, "undistorted_image_topic", undistorted_image_topic);
    getParam(node_handle, "debug_mode", debug_mode);
}

ImageUndistorterInterfaceParameters::ImageUndistorterInterfaceParameters() {
}

} // namespace stargazer_ros_tool
