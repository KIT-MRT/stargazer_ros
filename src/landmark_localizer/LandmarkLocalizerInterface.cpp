//
// Created by bandera on 09.06.16.
//

#include "LandmarkLocalizerInterface.h"
#include "../StargazerConversionMethods.h"
#include "stargazer/CeresLocalizer.h"
#include "stargazer/TriangulationLocalizer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "utils_ros/ros_console.hpp"

using namespace stargazer_ros_tool;

LandmarkLocalizerInterface::LandmarkLocalizerInterface(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle)
        : params_{LandmarkLocalizerInterfaceParameters::getInstance()} {

    // Set parameters
    params_.fromNodeHandle(private_node_handle);
    last_timestamp_ = ros::Time::now();
    debugVisualizer_.SetWaitTime(1);

    if (params_.use_ceres)
        localizer_ = std::make_unique<stargazer::CeresLocalizer>(params_.stargazer_config);
    else
        localizer_ = std::make_unique<stargazer::TriangulationLocalizer>(params_.stargazer_config);

    // Initialize publisher
    pose_pub = node_handle.advertise<geometry_msgs::PoseStamped>(params_.pose_topic, 1);
    lm_sub = private_node_handle.subscribe<stargazer_ros_tool::Landmarks>(
        params_.landmark_topic, 1, &LandmarkLocalizerInterface::landmarkCallback, this);

    if (params_.debug_mode)
        utils_ros::showNodeInfo();
}

void LandmarkLocalizerInterface::landmarkCallback(const stargazer_ros_tool::Landmarks::ConstPtr& msg) {

    ros::Time this_timestamp = msg->header.stamp;
    double dt = (this_timestamp - last_timestamp_).toSec();
    ros::Time last_timestamp = this_timestamp;

    std::vector<stargazer::ImgLandmark> detected_landmarks = convert2ImgLandmarks(*msg);

    // Localize
    localizer_->UpdatePose(detected_landmarks, dt);
    stargazer::pose_t pose = localizer_->getPose();

    // Publish tf pose
    tf::StampedTransform map2camTransform;
    pose2tf(pose, map2camTransform);
    map2camTransform.stamp_ = msg->header.stamp;
    map2camTransform.frame_id_ = params_.map_frame;
    map2camTransform.child_frame_id_ = params_.camera_frame;
    tf_pub.sendTransform(map2camTransform);

    geometry_msgs::PoseStamped poseStamped;
    poseStamped.header.frame_id = params_.map_frame;
    poseStamped.header.stamp = msg->header.stamp;
    poseStamped.pose = pose2gmPose(pose);
    pose_pub.publish(poseStamped);

    //  Visualize
    if (params_.debug_mode) {
        cv::Mat img = cv::Mat::zeros(1024, 1360, CV_8UC3);
        debugVisualizer_.DrawLandmarks(img, localizer_->getLandmarks(), localizer_->getIntrinsics(), pose);
        debugVisualizer_.DrawLandmarks(img, detected_landmarks);
        debugVisualizer_.ShowImage(img, "ReprojectionImage");
    }
}
