#include "LandmarkVisualizerParameters.h"

#include <utils_ros/node_handle.hpp>

namespace stargazer_ros_tool {

LandmarkVisualizerParameters& LandmarkVisualizerParameters::getInstance() {
    static LandmarkVisualizerParameters p;
    return p;
}

void LandmarkVisualizerParameters::fromNodeHandle(const ros::NodeHandle& node_handle) {
    using namespace utils_ros;

    getParam(node_handle, "stargazer_config", stargazer_config);
    getParam(node_handle, "landmark_topic", landmark_topic);
    getParam(node_handle, "rate", rate);
}

LandmarkVisualizerParameters::LandmarkVisualizerParameters() {
}

} // namespace stargazer_ros_tool
