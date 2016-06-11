#include "ReprojectionVisualizerParameters.h"

#include <utils_ros/node_handle.hpp>

namespace stargazer_ros_tool {

ReprojectionVisualizerParameters& ReprojectionVisualizerParameters::getInstance() {
    static ReprojectionVisualizerParameters p;
    return p;
}

void ReprojectionVisualizerParameters::fromNodeHandle(const ros::NodeHandle& node_handle) {
    using namespace utils_ros;

    getParam(node_handle, "stargazer_config", stargazer_config);
    getParam(node_handle, "bag_file", bag_file);
    getParam(node_handle, "landmark_topic", landmark_topic);
    getParam(node_handle, "img_topic", img_topic);
    getParam(node_handle, "pose_topic", pose_topic);
    getParam(node_handle, "waitTime", waitTime);
}

ReprojectionVisualizerParameters::ReprojectionVisualizerParameters() {
}

} // namespace stargazer_ros_tool
