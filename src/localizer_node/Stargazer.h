//
// Created by bandera on 28.03.16.
//

#pragma once

// ROS includes
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

// Image processing
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

// stargazer includes
#include "stargazer/StargazerTypes.h"
#include "stargazer/StargazerConfig.h"
#include "stargazer/LandmarkFinder.h"
#include "stargazer/Localizer.h"

// Msg formats
#include <geometry_msgs/PoseStamped.h>


class Stargazer {
 public:
  Stargazer(std::string cfgfile);

  ~Stargazer() { };

 private:
  // Node handles;
  ros::NodeHandle n, np;

  // Subscriber
  image_transport::Subscriber img_sub;
  image_transport::ImageTransport img_trans;

  // Publisher
  ros::Publisher pose_pub;
  tf::TransformBroadcaster tf_pub;

  LandmarkFinder landmarkFinder;
  Localizer localizer;


  void imgCallback(const sensor_msgs::ImageConstPtr &msg);
};


inline void pose2tf(const pose_t pose_in, tf::StampedTransform &transform) {
  transform.setOrigin(tf::Vector3(pose_in[(int) POSE::X],
                                  pose_in[(int) POSE::Y],
                                  pose_in[(int) POSE::Z]));
  double quaternion[4];
  double angleAxis[3];
  angleAxis[0] = pose_in[(int) POSE::Rx];
  angleAxis[1] = pose_in[(int) POSE::Ry];
  angleAxis[2] = pose_in[(int) POSE::Rz];
  ceres::AngleAxisToQuaternion(&angleAxis[0], &quaternion[0]);
  tf::Quaternion q;
  q.setW(quaternion[0]);
  q.setX(quaternion[1]);
  q.setY(quaternion[2]);
  q.setZ(quaternion[3]);
  transform.setRotation(q);
  return;
}