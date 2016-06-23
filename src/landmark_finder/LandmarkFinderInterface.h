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

// Image processing
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include "LandmarkFinderInterfaceParameters.h"
#include "stargazer/DebugVisualizer.h"
#include "stargazer/LandmarkFinder.h"
#include "stargazer_ros/LandmarkFinderConfig.h"

namespace stargazer_ros {

class LandmarkFinderInterface {

public:
    LandmarkFinderInterface(ros::NodeHandle, ros::NodeHandle);

private:
    // Subscriber
    image_transport::Subscriber img_sub;
    image_transport::ImageTransport img_trans;
    ros::Publisher lm_pub;
    dynamic_reconfigure::Server<LandmarkFinderConfig> server;

    LandmarkFinderInterfaceParameters& params_;
    stargazer::DebugVisualizer debugVisualizer_;

    std::unique_ptr<stargazer::LandmarkFinder> landmarkFinder;

    void imgCallback(const sensor_msgs::ImageConstPtr& msg);
    void reconfigureCallback(LandmarkFinderConfig& config, uint32_t level);
};

} // namespace stargazer_ros