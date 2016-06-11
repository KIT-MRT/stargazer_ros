#pragma once

#include <string>
#include <ros/node_handle.h>

namespace stargazer_ros_tool {

struct LandmarkFinderInterfaceParameters {

    static LandmarkFinderInterfaceParameters& getInstance();

    void fromNodeHandle(const ros::NodeHandle&);

    std::string stargazer_config;
    std::string landmark_topic;
    std::string undistorted_image_topic;

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
