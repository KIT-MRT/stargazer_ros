//
// Created by bandera on 09.06.16.
//

#pragma once

// ROS includes
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

// Msg formats
#include <geometry_msgs/PoseStamped.h>
#include "stargazer_ros_tool/LandmarkLocalizerConfig.h"
#include "stargazer_ros_tool/Landmarks.h"

#include "LandmarkLocalizerInterfaceParameters.h"
#include "stargazer/DebugVisualizer.h"
#include "stargazer/Localizer.h"
#include "stargazer/StargazerImgTypes.h"
#include "stargazer/StargazerTypes.h"

namespace stargazer_ros_tool {

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

    void landmarkCallback(const stargazer_ros_tool::Landmarks::ConstPtr& msg);
    void reconfigureCallback(LandmarkLocalizerConfig& config, uint32_t level);
};

} // namespace stargazer_ros_tool
