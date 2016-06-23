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

#include "LandmarkFinderInterface.h"
#include "../StargazerConversionMethods.h"
#include "../ros_utils.h"

using namespace stargazer_ros;

LandmarkFinderInterface::LandmarkFinderInterface(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle)
        : params_{LandmarkFinderInterfaceParameters::getInstance()}, img_trans(node_handle),
          server(private_node_handle) {

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

    // Setup dynamic reconfigure server
    server.setCallback(boost::bind(&LandmarkFinderInterface::reconfigureCallback, this, _1, _2));

    lm_pub = private_node_handle.advertise<stargazer_ros::LandmarkArray>(params_.landmark_topic, 1);
    img_sub = img_trans.subscribe(params_.undistorted_image_topic, 1, &LandmarkFinderInterface::imgCallback, this);

    debugVisualizer_.SetWaitTime(10);

    if (params_.debug_mode)
        showNodeInfo();
}

void LandmarkFinderInterface::imgCallback(const sensor_msgs::ImageConstPtr& msg) {

    cv_bridge::CvImagePtr cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    std::vector<stargazer::ImgLandmark> detected_img_landmarks;
    landmarkFinder->DetectLandmarks(cvPtr->image, detected_img_landmarks);

    // Convert
    stargazer_ros::LandmarkArray landmarksMessage = convert2LandmarkMsg(detected_img_landmarks, msg->header);
    lm_pub.publish(landmarksMessage);

    //  Visualize
    if (params_.debug_mode) {

        // Invert images
        cv::bitwise_not(landmarkFinder->rawImage_, landmarkFinder->rawImage_);
        cv::bitwise_not(landmarkFinder->grayImage_, landmarkFinder->grayImage_);
        cv::bitwise_not(landmarkFinder->filteredImage_, landmarkFinder->filteredImage_);

        // Show images
        debugVisualizer_.ShowImage(landmarkFinder->rawImage_, "Raw Image");
        debugVisualizer_.ShowImage(landmarkFinder->grayImage_, "Gray Image");
        debugVisualizer_.ShowImage(landmarkFinder->filteredImage_, "Filtered Image");

        // Show detections
        auto point_img = debugVisualizer_.ShowPoints(landmarkFinder->filteredImage_, landmarkFinder->clusteredPixels_);
        auto cluster_img =
            debugVisualizer_.ShowClusters(landmarkFinder->filteredImage_, landmarkFinder->clusteredPoints_);

        // Show landmarks
        cv::Mat temp = landmarkFinder->rawImage_.clone();
        debugVisualizer_.DrawLandmarks(temp, detected_img_landmarks);
        debugVisualizer_.ShowImage(temp, "Detected Landmarks");

        // clang-format off
//        static int count = 0;
//        cv::imwrite((boost::format("/home/bandera/Documents/MRT/Papers/ITSC2016/StargazerPaper/pics/raw_%010d.jpg") % count).str(),landmarkFinder->rawImage_);
//        cv::imwrite((boost::format("/home/bandera/Documents/MRT/Papers/ITSC2016/StargazerPaper/pics/gray_%010d.jpg") % count).str(),landmarkFinder->grayImage_);
//        cv::imwrite((boost::format("/home/bandera/Documents/MRT/Papers/ITSC2016/StargazerPaper/pics/filtered_%010d.jpg") % count).str(),landmarkFinder->filteredImage_);
//        cv::imwrite((boost::format("/home/bandera/Documents/MRT/Papers/ITSC2016/StargazerPaper/pics/points_%010d.jpg") % count).str(),point_img);
//        cv::imwrite((boost::format("/home/bandera/Documents/MRT/Papers/ITSC2016/StargazerPaper/pics/clusters_%010d.jpg") % count).str(),cluster_img);
//        cv::imwrite((boost::format("/home/bandera/Documents/MRT/Papers/ITSC2016/StargazerPaper/pics/landmarks_%010d.jpg") % count).str(),temp);
//        ++count;
        // clang-format on
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
