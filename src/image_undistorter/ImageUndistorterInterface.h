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

#include "stargazer/libCalibIO/CalibIO.hpp"

namespace stargazer_ros_tool {

class ImageUndistorterInterface {

public:
  ImageUndistorterInterface(ros::NodeHandle, ros::NodeHandle);

private:
  // Subscriber
  image_transport::Subscriber img_sub;
  image_transport::Publisher img_pub;
  image_transport::ImageTransport img_trans;

  cv::Mat m_oCalibMap_u;
  cv::Mat m_oCalibMap_v;

  void imgCallback(const sensor_msgs::ImageConstPtr &msg);
};

} // namespace stargazer_ros_tool