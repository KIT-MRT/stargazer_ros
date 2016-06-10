//
// Created by bandera on 10.06.16.
//

#pragma once

#include <stargazer_ros_tool/Landmark.h>
#include "LandmarkCalibratorInterfaceParameters.h"
#include "stargazer/BundleAdjuster.h"
#include "stargazer/StargazerTypes.h"

namespace stargazer_ros_tool {

class LandmarkCalibratorInterface {
public:
    LandmarkCalibratorInterface(ros::NodeHandle, ros::NodeHandle);

private:
    LandmarkCalibratorInterfaceParameters& params_;
    stargazer::BundleAdjuster bundleAdjuster;
    std::vector<std::array<double, 3>> observed_poses;
    std::vector<std::vector<stargazer::Landmark>> observed_landmarks;

    void load_data();
    void write_data();
    void optimize();
    stargazer::Landmark convert2Landmark(const stargazer_ros_tool::Landmark& lm_in);
};

} // namespace stargazer_ros_tool
