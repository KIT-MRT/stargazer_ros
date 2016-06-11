#pragma once

#include <ros/node_handle.h>
#include <string>

namespace stargazer_ros_tool {

struct LandmarkVisualizerParameters {

  static LandmarkVisualizerParameters &getInstance();

  void fromNodeHandle(const ros::NodeHandle &);

  std::string stargazer_config;
  std::string landmark_topic;
  double rate;

private:
  LandmarkVisualizerParameters();
};

} // namespace stargazer_ros_tool
