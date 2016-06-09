//
// Created by bandera on 09.06.16.
//

#pragma once

// ROS includes
#include <ros/ros.h>

// Image processing
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include "LandmarkFinderInterfaceParameters.h"
#include "stargazer/LandmarkFinder.h"

namespace stargazer_ros_tool {

class LandmarkFinderInterface {

public:
  LandmarkFinderInterface(ros::NodeHandle, ros::NodeHandle);

private:
  // Subscriber
  image_transport::Subscriber img_sub;
  image_transport::ImageTransport img_trans;
  ros::Publisher lm_pub;

  LandmarkFinderInterfaceParameters &params_;

  std::unique_ptr<LandmarkFinder> landmarkFinder;

  void imgCallback(const sensor_msgs::ImageConstPtr &msg);
};

} // namespace stargazer_ros_tool