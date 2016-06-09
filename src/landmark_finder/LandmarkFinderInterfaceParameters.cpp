#include "LandmarkFinderInterfaceParameters.h"

#include <utils_ros/node_handle.hpp>

namespace stargazer_ros_tool {

LandmarkFinderInterfaceParameters &
LandmarkFinderInterfaceParameters::getInstance() {
  static LandmarkFinderInterfaceParameters p;
  return p;
}

void LandmarkFinderInterfaceParameters::fromNodeHandle(
    const ros::NodeHandle &node_handle) {
  using namespace utils_ros;

  getParam(node_handle, "landmark_file", landmark_file);
  getParam(node_handle, "threshold", threshold);
  getParam(node_handle, "maxRadiusForPixelCluster", maxRadiusForPixelCluster);
  getParam(node_handle, "minPixelForCluster", minPixelForCluster);
  getParam(node_handle, "maxPixelForCluster", maxPixelForCluster);
  getParam(node_handle, "maxRadiusForCluster", maxRadiusForCluster);
  getParam(node_handle, "minPointsPerLandmark", minPointsPerLandmark);
  getParam(node_handle, "maxPointsPerLandmark", maxPointsPerLandmark);
  getParam(node_handle, "debug_mode", debug_mode);

}

LandmarkFinderInterfaceParameters::LandmarkFinderInterfaceParameters() {}

} // namespace stargazer_ros_tool
