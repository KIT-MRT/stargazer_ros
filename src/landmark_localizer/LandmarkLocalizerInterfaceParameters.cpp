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
  getParam(node_handle, "landmark_topic", landmark_topic);
  getParam(node_handle, "pose_topic", pose_topic);
  getParam(node_handle, "map_frame", map_frame);
  getParam(node_handle, "robot_frame", robot_frame);
  getParam(node_handle, "camera_frame", camera_frame);
  getParam(node_handle, "debug_mode", debug_mode);
}

LandmarkLocalizerInterfaceParameters::LandmarkLocalizerInterfaceParameters() {}

} // namespace stargazer_ros_tool
