//
// This file is part of the stargazer_ros package.
//
// Copyright 2016 Claudio Bandera <claudio.bandera@kit.edu (Karlsruhe Institute of Technology)
//
// The stargazer_ros package is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// The stargazer_ros package is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <http://www.gnu.org/licenses/>.

#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <rosbag/bag.h>
#include <stargazer_ros/Landmark.h>
#include "LandmarkCalibratorInterfaceParameters.h"
#include "stargazer/LandmarkCalibrator.h"
#include "stargazer/StargazerImgTypes.h"
#include "stargazer_ros/LandmarkArray.h"

namespace stargazer_ros {

class LandmarkCalibratorInterface {
public:
    LandmarkCalibratorInterface(ros::NodeHandle, ros::NodeHandle);
    ~LandmarkCalibratorInterface();

private:
    LandmarkCalibratorInterfaceParameters& params_;
    std::unique_ptr<stargazer::LandmarkCalibrator> bundleAdjuster;
    std::vector<stargazer::pose_t> observed_poses;
    std::vector<std::vector<stargazer::ImgLandmark>> observed_landmarks;
    std::vector<ros::Time> observed_timestamps;
    std::string pose_frame;
    rosbag::Bag bag_out;

    void load_data();
    void write_data();
    void optimize();
    void synchronizerCallback(const stargazer_ros::LandmarkArray::ConstPtr& lm_msg,
                              const geometry_msgs::PoseStamped::ConstPtr& pose_msg);
};

} // namespace stargazer_ros
