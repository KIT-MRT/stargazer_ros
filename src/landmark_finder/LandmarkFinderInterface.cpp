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
        : params_{LandmarkFinderInterfaceParameters::getInstance()}, img_trans(node_handle), server(ros::NodeHandle("/stargazer/LandmarkFinder")) {

    // Set parameters
    params_.fromNodeHandle(private_node_handle);
    landmarkFinder = std::make_unique<stargazer::LandmarkFinder>(params_.stargazer_config);
    landmarkFinder->threshold = static_cast<uint8_t>(params_.threshold);
    landmarkFinder->maxRadiusForCluster = params_.maxRadiusForCluster;
    landmarkFinder->maxRadiusForPixelCluster = params_.maxRadiusForPixelCluster;
    landmarkFinder->maxPixelForCluster = static_cast<uint16_t>(params_.maxPixelForCluster);
    landmarkFinder->minPixelForCluster = static_cast<uint16_t>(params_.minPixelForCluster);
    landmarkFinder->maxPointsPerLandmark = static_cast<uint16_t>(params_.maxPointsPerLandmark);
    landmarkFinder->minPointsPerLandmark = static_cast<uint16_t>(params_.minPointsPerLandmark);

    // Setup and set values in dynamic reconfigure server
    LandmarkFinderConfig config;
    config.debug_mode = params_.debug_mode;
    config.threshold = static_cast<uint8_t>(params_.threshold);
    config.maxRadiusForCluster = params_.maxRadiusForCluster;
    config.maxRadiusForPixelCluster = params_.maxRadiusForPixelCluster;
    config.maxPixelForCluster = static_cast<uint16_t>(params_.maxPixelForCluster);
    config.minPixelForCluster = static_cast<uint16_t>(params_.minPixelForCluster);
    config.maxPointsPerLandmark = static_cast<uint16_t>(params_.maxPointsPerLandmark);
    config.minPointsPerLandmark = static_cast<uint16_t>(params_.minPointsPerLandmark);
    server.updateConfig(config);
    server.setCallback(boost::bind(&LandmarkFinderInterface::reconfigureCallback, this, _1, _2));

    lm_pub = private_node_handle.advertise<stargazer_ros_tool::Landmarks>(params_.landmark_topic, 1);
    img_sub = img_trans.subscribe(params_.undistorted_image_topic, 1, &LandmarkFinderInterface::imgCallback, this);

    debugVisualizer_.SetWaitTime(10);

    if (params_.debug_mode)
        utils_ros::showNodeInfo();
}

void LandmarkFinderInterface::imgCallback(const sensor_msgs::ImageConstPtr& msg) {

    cv_bridge::CvImagePtr cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    std::vector<stargazer::ImgLandmark> detected_img_landmarks;
    landmarkFinder->DetectLandmarks(cvPtr->image, detected_img_landmarks);

    // Convert
    stargazer_ros_tool::Landmarks landmarksMessage = convert2LandmarkMsg(detected_img_landmarks, msg->header);
    lm_pub.publish(landmarksMessage);

    //  Visualize
    if (params_.debug_mode) {
        // TODO make debug Visualizer thread safe
        // Show images
        debugVisualizer_.ShowImage(landmarkFinder->rawImage_, "Raw Image");
        debugVisualizer_.ShowImage(landmarkFinder->grayImage_, "Gray Image");
        debugVisualizer_.ShowImage(landmarkFinder->filteredImage_, "Filtered Image");

        // Show detections
        debugVisualizer_.ShowPoints(landmarkFinder->filteredImage_, landmarkFinder->clusteredPixels_);
        debugVisualizer_.ShowClusters(landmarkFinder->filteredImage_, landmarkFinder->clusteredPoints_);

        // Show landmarks
        debugVisualizer_.DrawLandmarks(landmarkFinder->rawImage_, detected_img_landmarks);
        debugVisualizer_.ShowImage(landmarkFinder->rawImage_, "Detected Landmarks");
    }
}

void LandmarkFinderInterface::reconfigureCallback(LandmarkFinderConfig& config, uint32_t level) {
    params_.debug_mode = config.debug_mode;
    landmarkFinder->threshold = static_cast<uint8_t>(config.threshold);
    landmarkFinder->maxRadiusForCluster = config.maxRadiusForCluster;
    landmarkFinder->maxRadiusForPixelCluster = config.maxRadiusForPixelCluster;
    landmarkFinder->maxPixelForCluster = static_cast<uint16_t>(config.maxPixelForCluster);
    landmarkFinder->minPixelForCluster = static_cast<uint16_t>(config.minPixelForCluster);
    landmarkFinder->maxPointsPerLandmark = static_cast<uint16_t>(config.maxPointsPerLandmark);
    landmarkFinder->minPointsPerLandmark = static_cast<uint16_t>(config.minPointsPerLandmark);
}
