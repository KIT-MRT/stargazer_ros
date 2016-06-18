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
  std::string camera_frame;
  std::string landmark_topic;
  std::string pose_topic;
  bool debug_mode;

private:
  LandmarkLocalizerInterfaceParameters();
};

} // namespace stargazer_ros_tool
