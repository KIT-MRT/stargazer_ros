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

#include <memory>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include "ReprojectionVisualizerParameters.h"
#include "stargazer/DebugVisualizer.h"
#include "stargazer_ros/LandmarkArray.h"

namespace stargazer_ros {

class ReprojectionVisualizer {
public:
    ReprojectionVisualizer(ros::NodeHandle, ros::NodeHandle);

private:
    ReprojectionVisualizerParameters& params_;
    stargazer::landmark_map_t landmarks;
    stargazer::camera_params_t camera_intrinsics = {{0., 0., 0., 0., 0., 0.}};

    std::unique_ptr<stargazer::DebugVisualizer> debugVisualizer_;

    void synchronizerCallback(const stargazer_ros::LandmarkArray::ConstPtr& lm_msg,
                              const geometry_msgs::PoseStamped::ConstPtr& pose_msg,
                              const sensor_msgs::ImageConstPtr& img_msg);
};

} // namespace stargazer_ros
