#include "LandmarkLocalizerInterfaceParameters.h"

#include <utils_ros/node_handle.hpp>

namespace stargazer_ros_tool {

LandmarkLocalizerInterfaceParameters &
LandmarkLocalizerInterfaceParameters::getInstance() {
  static LandmarkLocalizerInterfaceParameters p;
  return p;
}

void LandmarkLocalizerInterfaceParameters::fromNodeHandle(
    const ros::NodeHandle &node_handle) {
  using namespace utils_ros;

  getParam(node_handle, "stargazer_config", stargazer_config);
  getParam(node_handle, "map_frame", map_frame);
  getParam(node_handle, "robot_frame", robot_frame);
}

LandmarkLocalizerInterfaceParameters::LandmarkLocalizerInterfaceParameters() {}

} // namespace stargazer_ros_tool
