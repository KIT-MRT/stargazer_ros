//
// Created by bandera on 09.06.16.
//

#pragma once

// ROS includes
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

// Msg formats
#include <geometry_msgs/PoseStamped.h>
#include "stargazer_ros_tool/Landmarks.h"

#include <stargazer/Localizer.h>
#include <stargazer/StargazerImgTypes.h>
#include "LandmarkLocalizerInterfaceParameters.h"
#include "stargazer/CeresLocalizer.h"
#include "stargazer/StargazerTypes.h"
#include "stargazer/DebugVisualizer.h"

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

    LandmarkLocalizerInterfaceParameters& params_;
    stargazer::DebugVisualizer debugVisualizer;

    std::unique_ptr<stargazer::Localizer> triangLocalizer;
    std::unique_ptr<stargazer::CeresLocalizer> ceresLocalizer;

    ros::Time last_timestamp;

    void landmarkCallback(const stargazer_ros_tool::Landmarks::ConstPtr& msg);
};

} // namespace stargazer_ros_tool

inline void pose2tf(const stargazer::pose_t pose_in, tf::StampedTransform& transform) {
    using namespace stargazer;
    transform.setOrigin(tf::Vector3(pose_in[(int)POSE::X], pose_in[(int)POSE::Y], pose_in[(int)POSE::Z]));
    double quaternion[4];
    double angleAxis[3];
    angleAxis[0] = pose_in[(int)POSE::Rx];
    angleAxis[1] = pose_in[(int)POSE::Ry];
    angleAxis[2] = pose_in[(int)POSE::Rz];
    ceres::AngleAxisToQuaternion(&angleAxis[0], &quaternion[0]);
    tf::Quaternion q;
    q.setW(quaternion[0]);
    q.setX(quaternion[1]);
    q.setY(quaternion[2]);
    q.setZ(quaternion[3]);
    transform.setRotation(q);
    return;
}
