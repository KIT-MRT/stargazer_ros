#pragma once

#include <string>
#include <ros/node_handle.h>

namespace stargazer_ros_tool {

struct ImageUndistorterInterfaceParameters {

    static ImageUndistorterInterfaceParameters& getInstance();

    void fromNodeHandle(const ros::NodeHandle&);

    std::string calib_file;
    std::string raw_image_topic;
    std::string undistorted_image_topic;
    bool debug_mode;

private:
    ImageUndistorterInterfaceParameters();
};

} // namespace stargazer_ros_tool
