#pragma once

#include <ros/node_handle.h>
#include <string>

namespace stargazer_ros_tool {

struct LandmarkFinderInterfaceParameters {

  static LandmarkFinderInterfaceParameters &getInstance();

  void fromNodeHandle(const ros::NodeHandle &);

  std::string landmark_file;
  int threshold;
  float maxRadiusForPixelCluster;
  int minPixelForCluster;
  int maxPixelForCluster;
  float maxRadiusForCluster;
  int minPointsPerLandmark;
  int maxPointsPerLandmark;
  bool debug_mode;

private:
  LandmarkFinderInterfaceParameters();
};

} // namespace stargazer_ros_tool
