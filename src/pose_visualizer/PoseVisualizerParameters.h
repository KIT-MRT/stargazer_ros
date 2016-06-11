#pragma once

#include <string>
#include <ros/node_handle.h>

namespace stargazer_ros_tool {

struct PoseVisualizerParameters {

    static PoseVisualizerParameters& getInstance();

    void fromNodeHandle(const ros::NodeHandle&);

    std::string bag_file;
    std::string world_frame;
    std::string pose_topic;
    std::string pose_optimized_topic;
    std::string pose_pub_topic;
    std::string pose_opt_pub_topic;
    double rate;

private:
    PoseVisualizerParameters();
};

} // namespace stargazer_ros_tool
