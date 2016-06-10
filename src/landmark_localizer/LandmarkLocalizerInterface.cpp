//
// Created by bandera on 09.06.16.
//

#include "LandmarkLocalizerInterface.h"

// stargazer includes
#include "stargazer/StargazerConfig.h"
#include "stargazer_ros_tool/Landmarks.h"

using namespace stargazer_ros_tool;

LandmarkLocalizerInterface::LandmarkLocalizerInterface(
    ros::NodeHandle node_handle, ros::NodeHandle private_node_handle)
    : params_{LandmarkLocalizerInterfaceParameters::getInstance()} {

  // Set parameters
  params_.fromNodeHandle(private_node_handle);

  landmarkLocalizer = std::make_unique<stargazer::Localizer>(params_.landmark_file);

  // Initialize publisher
  pose_pub =
      private_node_handle.advertise<geometry_msgs::PoseStamped>("pose", 1);
  lm_sub = private_node_handle.subscribe<stargazer_ros_tool::Landmarks>(
      "/landmarks_seen", 1, &LandmarkLocalizerInterface::landmarkCallback,
      this);
}

void LandmarkLocalizerInterface::landmarkCallback(
    const stargazer_ros_tool::LandmarksConstPtr &msg) {
  ROS_DEBUG_STREAM("Landmark Callback, received " << msg->landmarks.size()
                                                  << " landmarks");

  // Convert
  std::vector<stargazer::Landmark> detected_landmarks;
  detected_landmarks.reserve(msg->landmarks.size());
  for (auto &el : msg->landmarks)
    detected_landmarks.push_back(convert2Landmark(el));

  // Localize
  landmarkLocalizer->UpdatePose(detected_landmarks);

  // Publish tf pose
  tf::StampedTransform transform;
  pose2tf(landmarkLocalizer->getPose(), transform);
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
