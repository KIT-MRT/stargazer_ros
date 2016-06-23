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

// ROS includes
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

// Msg formats
#include <geometry_msgs/PoseStamped.h>
#include "stargazer_ros/LandmarkArray.h"
#include "stargazer_ros/LandmarkLocalizerConfig.h"

#include "LandmarkLocalizerInterfaceParameters.h"
#include "stargazer/DebugVisualizer.h"
#include "stargazer/Localizer.h"
#include "stargazer/StargazerImgTypes.h"
#include "stargazer/StargazerTypes.h"

namespace stargazer_ros {

class LandmarkLocalizerInterface {

public:
    LandmarkLocalizerInterface(ros::NodeHandle, ros::NodeHandle);

private:
    // Subscriber
    ros::Subscriber lm_sub;

    // Publisher
    ros::Publisher pose_pub;
    tf::TransformBroadcaster tf_pub;
    tf::StampedTransform camRobotTransform;
    dynamic_reconfigure::Server<LandmarkLocalizerConfig> server;

    LandmarkLocalizerInterfaceParameters& params_;
    stargazer::DebugVisualizer debugVisualizer_;

    std::unique_ptr<stargazer::Localizer> localizer_;

    ros::Time last_timestamp_;

    void landmarkCallback(const stargazer_ros::LandmarkArray::ConstPtr& msg);
    void reconfigureCallback(LandmarkLocalizerConfig& config, uint32_t level);
};

} // namespace stargazer_ros
