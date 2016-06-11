//
// Created by bandera on 09.06.16.
//

#include "LandmarkFinderInterface.h"
#include <utils_ros/ros_console.hpp>
#include "../StargazerConversionMethods.h"
#include "stargazer/StargazerImgTypes.h"
#include "stargazer_ros_tool/Landmarks.h"

using namespace stargazer_ros_tool;

LandmarkFinderInterface::LandmarkFinderInterface(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle)
        : params_{LandmarkFinderInterfaceParameters::getInstance()}, img_trans(node_handle) {

    // Set parameters
    params_.fromNodeHandle(private_node_handle);
    landmarkFinder = std::make_unique<stargazer::LandmarkFinder>(params_.stargazer_config);
    landmarkFinder->debug_mode = params_.debug_mode;
    landmarkFinder->m_cThreshold = static_cast<uint8_t>(params_.threshold);
    landmarkFinder->m_fMaxRadiusForCluster = params_.maxRadiusForCluster;
    landmarkFinder->m_fMaxRadiusForPixelCluster = params_.maxRadiusForPixelCluster;
    landmarkFinder->m_nMaxPixelForCluster = static_cast<uint16_t>(params_.maxPixelForCluster);
    landmarkFinder->m_nMinPixelForCluster = static_cast<uint16_t>(params_.minPixelForCluster);
    landmarkFinder->m_nMaxPointsPerLandmark = static_cast<uint16_t>(params_.maxPointsPerLandmark);
    landmarkFinder->m_nMinPointsPerLandmark = static_cast<uint16_t>(params_.minPointsPerLandmark);

    lm_pub = private_node_handle.advertise<stargazer_ros_tool::Landmarks>(params_.landmark_topic, 1);
    img_sub = img_trans.subscribe(params_.undistorted_image_topic, 1, &LandmarkFinderInterface::imgCallback, this);

    utils_ros::showNodeInfo();
}

void LandmarkFinderInterface::imgCallback(const sensor_msgs::ImageConstPtr& msg) {

    cv_bridge::CvImagePtr cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    std::vector<stargazer::ImgLandmark> detected_img_landmarks;
    landmarkFinder->FindLandmarks(cvPtr->image, detected_img_landmarks);

    // Convert
    stargazer_ros_tool::Landmarks landmarksMessage = convert2LandmarkMsg(detected_img_landmarks, msg->header);
    lm_pub.publish(landmarksMessage);
}