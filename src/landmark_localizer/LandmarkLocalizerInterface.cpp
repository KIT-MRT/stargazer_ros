//
// Created by bandera on 09.06.16.
//

#include "LandmarkLocalizerInterface.h"
#include "../StargazerMessageAdapters.h"

using namespace stargazer_ros_tool;

LandmarkLocalizerInterface::LandmarkLocalizerInterface(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle)
        : params_{LandmarkLocalizerInterfaceParameters::getInstance()} {

    // Set parameters
    params_.fromNodeHandle(private_node_handle);
    last_timestamp = ros::Time::now();

    if (params_.use_ceres) {
        ceresLocalizer = std::make_unique<stargazer::CeresLocalizer>(params_.stargazer_config);
    } else {
        triangLocalizer = std::make_unique<stargazer::Localizer>(params_.stargazer_config);
    }

    // Initialize publisher
    pose_pub = private_node_handle.advertise<geometry_msgs::PoseStamped>("pose", 1);
    lm_sub = private_node_handle.subscribe<stargazer_ros_tool::Landmarks>(
        "/landmarks_seen", 1, &LandmarkLocalizerInterface::landmarkCallback, this);
}

void LandmarkLocalizerInterface::landmarkCallback(const stargazer_ros_tool::Landmarks::ConstPtr& msg) {
    ROS_DEBUG_STREAM("Landmark Callback, received " << msg->landmarks.size() << " landmarks");
    ros::Time this_timestamp = msg->header.stamp;

    stargazer::pose_t pose;
    if (params_.use_ceres) {
        // Convert
        std::vector<stargazer::Landmark> detected_landmarks = convert2Landmarks(*msg);

        // Localize
        ceresLocalizer->UpdatePose(detected_landmarks);
        pose = ceresLocalizer->getPose();

        //  Visualize
        if (params_.debug_mode) {
            cv::Mat img = cv::Mat::zeros(1024, 1360, CV_8UC3);
            std::vector<stargazer::ImgLandmark> detected_img_landmarks = convert2ImgLandmarks(*msg);
            debugVisualizer.DrawLandmarks(img, ceresLocalizer->getLandmarks(), ceresLocalizer->getIntrinsics(), pose);
            debugVisualizer.DrawLandmarks(img, detected_img_landmarks);
            debugVisualizer.ShowImage(img, "ReprojectionImage");
        }
    } else {
        std::vector<stargazer::ImgLandmark> detected_landmarks = convert2ImgLandmarks(*msg);

        double dt = (this_timestamp - last_timestamp).toSec();
        ros::Time last_timestamp = this_timestamp;

        // Localize
        triangLocalizer->UpdatePose(detected_landmarks, dt);
        pose = triangLocalizer->getPose();

        //  Visualize
        if (params_.debug_mode) {
            cv::Mat img = cv::Mat::zeros(1024, 1360, CV_8UC3);
            debugVisualizer.DrawLandmarks(img, triangLocalizer->getLandmarks(), triangLocalizer->getIntrinsics(), pose);
            debugVisualizer.DrawLandmarks(img, detected_landmarks);
            debugVisualizer.ShowImage(img, "ReprojectionImage");
        }
    }

    // Publish tf pose
    tf::StampedTransform transform;
    pose2tf(pose, transform);
    transform.stamp_ = msg->header.stamp;
    transform.frame_id_ = params_.map_frame;
    transform.child_frame_id_ = params_.robot_frame;
    tf_pub.sendTransform(transform);

    geometry_msgs::PoseStamped poseStamped;
    poseStamped.header.frame_id = params_.robot_frame;
    poseStamped.header.stamp = msg->header.stamp;
    poseStamped.pose.orientation.w = 1;
    pose_pub.publish(poseStamped);
}
