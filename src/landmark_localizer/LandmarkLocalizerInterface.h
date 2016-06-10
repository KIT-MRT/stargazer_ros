//
// Created by bandera on 09.06.16.
//

#pragma once

// ROS includes
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

// Msg formats
#include "stargazer_ros_tool/Landmarks.h"
#include <geometry_msgs/PoseStamped.h>

#include "LandmarkLocalizerInterfaceParameters.h"
#include "stargazer/Localizer.h"
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

  LandmarkLocalizerInterfaceParameters &params_;

  std::unique_ptr<Localizer> landmarkLocalizer;

  void landmarkCallback(const stargazer_ros_tool::LandmarksConstPtr &msg);
};

} // namespace stargazer_ros_tool

inline void pose2tf(const pose_t pose_in, tf::StampedTransform &transform) {
  transform.setOrigin(tf::Vector3(pose_in[(int)POSE::X], pose_in[(int)POSE::Y],
                                  pose_in[(int)POSE::Z]));
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

inline Landmark convert2Landmark(const stargazer_ros_tool::Landmark &lm_in) {
  Landmark lm_out(lm_in.id);
  lm_out.points.clear();
  lm_out.points.reserve(lm_in.corner_points.size() + lm_in.id_points.size());

  for (auto &el : lm_in.corner_points) {
    Point pt = {(double)el.u, (double)el.v, 0};
    lm_out.points.push_back(pt);
  }
  for (auto &el : lm_in.id_points) {
    Point pt = {(double)el.u, (double)el.v, 0};
    lm_out.points.push_back(pt);
  }

  return lm_out;
};
