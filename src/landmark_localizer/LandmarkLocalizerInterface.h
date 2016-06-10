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

    std::unique_ptr<stargazer::Localizer> triangLocalizer;
    std::unique_ptr<stargazer::CeresLocalizer> ceresLocalizer;

    ros::Time last_timestamp;

    void landmarkCallback(const stargazer_ros_tool::LandmarksConstPtr& msg);
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

inline stargazer::Landmark convert2Landmark(const stargazer_ros_tool::Landmark& lm_in) {
    stargazer::Landmark lm_out(lm_in.id);
    lm_out.points.clear();
    lm_out.points.reserve(lm_in.corner_points.size() + lm_in.id_points.size());

    for (auto& el : lm_in.corner_points) {
        stargazer::Point pt = {(double)el.u, (double)el.v, 0};
        lm_out.points.push_back(pt);
    }
    for (auto& el : lm_in.id_points) {
        stargazer::Point pt = {(double)el.u, (double)el.v, 0};
        lm_out.points.push_back(pt);
    }

    return lm_out;
};

inline stargazer::ImgLandmark convert2ImgLandmark(const stargazer_ros_tool::Landmark& lm_in) {
    stargazer::ImgLandmark lm_out;
    lm_out.nID = lm_in.id;

    lm_out.voCorners.reserve(lm_in.corner_points.size());
    for (auto& el : lm_in.corner_points) {
        cv::Point pt;
        pt.x = el.u;
        pt.y = el.v;
        lm_out.voCorners.push_back(pt);
    }

    lm_out.voIDPoints.reserve(lm_in.id_points.size());
    for (auto& el : lm_in.id_points) {
        cv::Point pt;
        pt.x = el.u;
        pt.y = el.v;
        lm_out.voIDPoints.push_back(pt);
    }

    return lm_out;
};
