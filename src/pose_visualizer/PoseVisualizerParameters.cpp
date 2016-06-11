#include "PoseVisualizerParameters.h"

#include <utils_ros/node_handle.hpp>

namespace stargazer_ros_tool {

PoseVisualizerParameters& PoseVisualizerParameters::getInstance() {
    static PoseVisualizerParameters p;
    return p;
}

void PoseVisualizerParameters::fromNodeHandle(const ros::NodeHandle& node_handle) {
    using namespace utils_ros;

    getParam(node_handle, "bag_file", bag_file);
    getParam(node_handle, "world_frame", world_frame);
    getParam(node_handle, "pose_topic", pose_topic);
    getParam(node_handle, "pose_optimized_topic", pose_optimized_topic);
    getParam(node_handle, "pose_pub_topic", pose_pub_topic);
    getParam(node_handle, "pose_opt_pub_topic", pose_opt_pub_topic);
    getParam(node_handle, "rate", rate);
}

PoseVisualizerParameters::PoseVisualizerParameters() {
}

} // namespace stargazer_ros_tool
