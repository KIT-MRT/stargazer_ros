#include "LandmarkFinderInterfaceParameters.h"

#include "../ros_utils.h"

namespace stargazer_ros_tool {

LandmarkFinderInterfaceParameters& LandmarkFinderInterfaceParameters::getInstance() {
    static LandmarkFinderInterfaceParameters p;
    return p;
}

void LandmarkFinderInterfaceParameters::fromNodeHandle(const ros::NodeHandle& node_handle) {
    

    getParam(node_handle, "stargazer_config", stargazer_config);
    getParam(node_handle, "landmark_topic", landmark_topic);
    getParam(node_handle, "undistorted_image_topic", undistorted_image_topic);
    getParam(node_handle, "threshold", threshold);
    getParam(node_handle, "tight_filter_size", tight_filter_size);
    getParam(node_handle, "wide_filter_size", wide_filter_size);
    getParam(node_handle, "maxRadiusForPixelCluster", maxRadiusForPixelCluster);
    getParam(node_handle, "minPixelForCluster", minPixelForCluster);
    getParam(node_handle, "maxPixelForCluster", maxPixelForCluster);
    getParam(node_handle, "maxRadiusForCluster", maxRadiusForCluster);
    getParam(node_handle, "minPointsPerLandmark", minPointsPerLandmark);
    getParam(node_handle, "maxPointsPerLandmark", maxPointsPerLandmark);
    getParam(node_handle, "debug_mode", debug_mode);
}

LandmarkFinderInterfaceParameters::LandmarkFinderInterfaceParameters() {
}

} // namespace stargazer_ros_tool
