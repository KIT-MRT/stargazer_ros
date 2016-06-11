//
// Created by bandera on 10.06.16.
//

#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <stargazer_ros_tool/Landmark.h>
#include <rosbag/bag.h>
#include "LandmarkCalibratorInterfaceParameters.h"
#include "stargazer/BundleAdjuster.h"
#include "stargazer/StargazerImgTypes.h"
#include "stargazer_ros_tool/Landmarks.h"

namespace stargazer_ros_tool {

class LandmarkCalibratorInterface {
public:
    LandmarkCalibratorInterface(ros::NodeHandle, ros::NodeHandle);

private:
    LandmarkCalibratorInterfaceParameters& params_;
    stargazer::BundleAdjuster bundleAdjuster;
    std::vector<std::array<double, 3>> observed_poses;
    std::vector<std::vector<stargazer::ImgLandmark>> observed_landmarks;
    std::vector<ros::Time> observed_timestamps;
    std::string pose_frame;
    rosbag::Bag bag_out;

    void load_data();
    void write_data();
    void optimize();
    void synchronizerCallback(const stargazer_ros_tool::Landmarks::ConstPtr& lm_msg,
                              const geometry_msgs::PoseStamped::ConstPtr& pose_msg);
};

} // namespace stargazer_ros_tool
