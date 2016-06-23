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

#include "ReprojectionVisualizer.h"

// ROS includes
#include <rosbag/bag.h>
#include <rosbag/view.h>

// ROS messages
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

// Local Helpers
#include <boost/foreach.hpp>
#include <tf/transform_datatypes.h>
#include "../StargazerConversionMethods.h"
#include "../ros_utils.h"
#include "stargazer/CoordinateTransformations.h"
#include "stargazer/StargazerConfig.h"
#define foreach BOOST_FOREACH

using namespace stargazer_ros;
using namespace stargazer;

/**
 * Inherits from message_filters::SimpleFilter<M>
 * to use protected signalMessage function
 */
template <class M>
class BagSubscriber : public message_filters::SimpleFilter<M> {
public:
    void newMessage(const boost::shared_ptr<M const>& msg) {
        this->signalMessage(msg);
    }
};

ReprojectionVisualizer::ReprojectionVisualizer(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle)
        : params_{ReprojectionVisualizerParameters::getInstance()} {

    showNodeInfo();

    // Set parameters
    params_.fromNodeHandle(private_node_handle);
    readConfig(params_.stargazer_config, camera_intrinsics, landmarks);

    // Convert landmark points to worldcoordinates once.
    for (auto& el : landmarks) {
        for (auto& pt : el.second.points) {
            double x, y, z;
            transformLM2World(&pt[(int)POINT::X], &pt[(int)POINT::Y], el.second.pose.data(), &x, &y, &z);
            pt[(int)POINT::X] = x;
            pt[(int)POINT::Y] = y;
            pt[(int)POINT::Z] = z;
        }
    }

    debugVisualizer_ = std::make_unique<DebugVisualizer>();
    debugVisualizer_->SetWaitTime(params_.waitTime);

    /*
     * Read Bag
     */
    rosbag::Bag bag(params_.bag_file, rosbag::bagmode::Read);

    std::vector<std::string> topics;
    topics.push_back(std::string(params_.landmark_topic));
    topics.push_back(std::string(params_.img_topic));
    topics.push_back(std::string(params_.pose_topic));

    // Set up fake subscribers to capture images
    BagSubscriber<stargazer_ros::LandmarkArray> lm_sub;
    BagSubscriber<geometry_msgs::PoseStamped> pose_sub;
    BagSubscriber<sensor_msgs::Image> img_sub;

    // Use time synchronizer to make sure we get properly synchronized images
    message_filters::TimeSynchronizer<stargazer_ros::LandmarkArray, geometry_msgs::PoseStamped, sensor_msgs::Image>
        sync(lm_sub, pose_sub, img_sub, 25);
    sync.registerCallback(boost::bind(&ReprojectionVisualizer::synchronizerCallback, this, _1, _2, _3));

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    foreach (rosbag::MessageInstance const m, view) {

        if (m.isType<stargazer_ros::LandmarkArray>()) {
            stargazer_ros::LandmarkArray::ConstPtr lm_msg = m.instantiate<stargazer_ros::LandmarkArray>();
            lm_sub.newMessage(lm_msg);
        } else if (m.isType<geometry_msgs::PoseStamped>()) {
            geometry_msgs::PoseStamped::ConstPtr pose_msg = m.instantiate<geometry_msgs::PoseStamped>();
            pose_sub.newMessage(pose_msg);
        } else if (m.isType<sensor_msgs::Image>()) {
            sensor_msgs::ImageConstPtr img_msg = m.instantiate<sensor_msgs::Image>();
            img_sub.newMessage(img_msg);
        }

        ros::spinOnce();
        if (!ros::ok())
            break;
    }

    bag.close();
}

void ReprojectionVisualizer::synchronizerCallback(const stargazer_ros::LandmarkArray::ConstPtr& lm_msg,
                                                  const geometry_msgs::PoseStamped::ConstPtr& pose_msg,
                                                  const sensor_msgs::ImageConstPtr& img_msg) {

    cv_bridge::CvImagePtr cvPtr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);

    std::vector<stargazer::ImgLandmark> img_lms = convert2ImgLandmarks(*lm_msg);
    debugVisualizer_->DrawLandmarks(cvPtr->image, landmarks, camera_intrinsics, gmPose2pose(pose_msg->pose));
    debugVisualizer_->DrawLandmarks(cvPtr->image, img_lms);
    debugVisualizer_->ShowImage(cvPtr->image);
}
