//
// Created by bandera on 09.06.16.
//

#include "LandmarkFinderInterface.h"
#include "stargazer_ros_tool/Landmarks.h"
#include "stargazer/StargazerImgTypes.h"

using namespace stargazer_ros_tool;

LandmarkFinderInterface::LandmarkFinderInterface(
    ros::NodeHandle node_handle, ros::NodeHandle private_node_handle)
    : params_{LandmarkFinderInterfaceParameters::getInstance()},
      img_trans(node_handle) {

  // Set parameters
  params_.fromNodeHandle(private_node_handle);
  landmarkFinder = std::make_unique<LandmarkFinder>(params_.landmark_file);
  landmarkFinder->debug_mode = params_.debug_mode;
  landmarkFinder->m_cThreshold = static_cast<uint8_t>(params_.threshold);
  landmarkFinder->m_fMaxRadiusForCluster = params_.maxRadiusForCluster;
  landmarkFinder->m_fMaxRadiusForPixelCluster = params_.maxRadiusForPixelCluster;
  landmarkFinder->m_nMaxPixelForCluster = static_cast<uint8_t>(params_.maxPixelForCluster);
  landmarkFinder->m_nMinPixelForCluster = static_cast<uint8_t>(params_.minPixelForCluster);
  landmarkFinder->m_nMaxPointsPerLandmark = static_cast<uint8_t>(params_.maxPointsPerLandmark);
  landmarkFinder->m_nMinPointsPerLandmark = static_cast<uint8_t>(params_.minPointsPerLandmark);

  lm_pub = private_node_handle.advertise<stargazer_ros_tool::Landmarks>(
      "/landmarks_seen", 1);
  img_sub = img_trans.subscribe("/image_undistort", 1,
                                &LandmarkFinderInterface::imgCallback, this);

}

void LandmarkFinderInterface::imgCallback(
    const sensor_msgs::ImageConstPtr &msg) {

  cv_bridge::CvImagePtr cvPtr =
      cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

  std::vector<ImgLandmark> detected_img_landmarks;
  landmarkFinder->FindLandmarks(cvPtr->image, detected_img_landmarks);

  // Convert
  stargazer_ros_tool::Landmarks landmarksMessage;
  landmarksMessage.header = msg->header;
  for(auto& lm : detected_img_landmarks) {
    stargazer_ros_tool::Landmark landmark;
    landmark.header = msg->header;
    landmark.id = lm.nID;
    for (auto& pt : lm.voCorners){
      stargazer_ros_tool::LandmarkPoint lmpt;
      lmpt.u = static_cast<uint8_t>(pt.x);
      lmpt.v = static_cast<uint8_t>(pt.y);
      landmark.corner_points.push_back(lmpt);
    }
    for (auto& pt : lm.voIDPoints){
      stargazer_ros_tool::LandmarkPoint lmpt;
      lmpt.u = static_cast<uint8_t>(pt.x);
      lmpt.v = static_cast<uint8_t>(pt.y);
      landmark.id_points.push_back(lmpt);
    }
    landmarksMessage.landmarks.push_back(landmark);
  }
  lm_pub.publish(landmarksMessage);
}