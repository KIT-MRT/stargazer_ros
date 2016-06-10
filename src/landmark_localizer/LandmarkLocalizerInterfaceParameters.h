#pragma once

#include <ros/node_handle.h>
#include <string>

namespace stargazer_ros_tool {

struct LandmarkLocalizerInterfaceParameters {

  static LandmarkLocalizerInterfaceParameters &getInstance();

  void fromNodeHandle(const ros::NodeHandle &);

  std::string stargazer_config;
  std::string map_frame;
  std::string robot_frame;
  bool use_ceres;
  bool debug_mode;

private:
  LandmarkLocalizerInterfaceParameters();
};

} // namespace stargazer_ros_tool
