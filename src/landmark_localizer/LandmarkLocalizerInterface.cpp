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

#include "LandmarkLocalizerInterface.h"
#include "../StargazerConversionMethods.h"
#include "../ros_utils.h"
#include "stargazer/CeresLocalizer.h"

using namespace stargazer_ros;

LandmarkLocalizerInterface::LandmarkLocalizerInterface(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle)
        : params_{LandmarkLocalizerInterfaceParameters::getInstance()}, server(private_node_handle) {

    // Set parameters
    params_.fromNodeHandle(private_node_handle);
    last_timestamp_ = ros::Time::now();
    debugVisualizer_.SetWaitTime(1);

    // Setup and set values in dynamic reconfigure server
    server.setCallback(boost::bind(&LandmarkLocalizerInterface::reconfigureCallback, this, _1, _2));

    localizer_ = std::make_unique<stargazer::CeresLocalizer>(params_.stargazer_config);

    // Initialize publisher
    pose_pub = node_handle.advertise<geometry_msgs::PoseStamped>(params_.pose_topic, 1);
    lm_sub = private_node_handle.subscribe<stargazer_ros::LandmarkArray>(
        params_.landmark_topic, 1, &LandmarkLocalizerInterface::landmarkCallback, this);

    if (params_.debug_mode)
        showNodeInfo();
}

void LandmarkLocalizerInterface::landmarkCallback(const stargazer_ros::LandmarkArray::ConstPtr& msg) {

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
        img.setTo(cv::Scalar(255, 255, 255));
        debugVisualizer_.DrawLandmarks(img, detected_landmarks);
        debugVisualizer_.DrawLandmarks(img, localizer_->getLandmarks(), localizer_->getIntrinsics(), pose);
        debugVisualizer_.ShowImage(img, "ReprojectionImage");

        // clang-format off
//        static int count = 0;
//        cv::imwrite((boost::format("/home/bandera/Documents/MRT/Papers/ITSC2016/StargazerPaper/pics/reprojection_%010d.jpg") % count).str(),img);
//        ++count;
        // clang-format on
    }

    const ceres::Solver::Summary& summary = dynamic_cast<stargazer::CeresLocalizer*>(localizer_.get())->getSummary();
    ROS_DEBUG_STREAM("Number of iterations: " << summary.iterations.size()
                                              << " Time needed: " << summary.total_time_in_seconds);
    if (summary.termination_type != ceres::TerminationType::CONVERGENCE) {
        ROS_WARN_STREAM("Solver did not converge! " << ceres::TerminationTypeToString(summary.termination_type));
        ROS_WARN_STREAM(summary.FullReport());
    }
}

void LandmarkLocalizerInterface::reconfigureCallback(LandmarkLocalizerConfig& config, uint32_t level) {
    params_.debug_mode = config.debug_mode;
}