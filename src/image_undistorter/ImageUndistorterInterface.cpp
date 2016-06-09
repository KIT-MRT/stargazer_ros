//
// Created by bandera on 09.06.16.
//

#include "ImageUndistorterInterface.h"

using namespace stargazer_ros_tool;

ImageUndistorterInterface::ImageUndistorterInterface(
    ros::NodeHandle node_handle, ros::NodeHandle private_node_handle)
    : img_trans(node_handle) {

  std::string calib_file;
  if (!private_node_handle.getParam("calib_file", calib_file))
    throw std::runtime_error("Missing parameter: calib_file");

  CalibIO calibration;
  if (!calibration.readCalibFromFile(calib_file)) {
    throw std::runtime_error("cannot read calibration file");
  }
  calibration.computeLUT(0, 0, m_oCalibMap_u, m_oCalibMap_v);

  img_sub = img_trans.subscribe("/image_raw", 1,
                                &ImageUndistorterInterface::imgCallback, this);
  img_pub = img_trans.advertise("/image_undistort", 1);
}

void ImageUndistorterInterface::imgCallback(
    const sensor_msgs::ImageConstPtr &msg) {
  cv_bridge::CvImagePtr cvPtr =
      cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  cv::Mat inImage = cvPtr->image;
  cv_bridge::CvImage outImage;
  outImage.header = cvPtr->header;
  outImage.encoding = cvPtr->encoding;
  cv::remap(inImage, outImage.image, m_oCalibMap_u, m_oCalibMap_v,
            cv::INTER_LINEAR);
  sensor_msgs::ImagePtr pData(outImage.toImageMsg());
  img_pub.publish(pData);
}