//
// Created by bandera on 10.06.16.
//

#include "LandmarkCalibratorInterface.h"

// ROS includes
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

// ROS messages
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

// Local Helpers
#include <boost/foreach.hpp>
#include <tf/transform_datatypes.h>
#include "../StargazerMessageAdapters.h"
#include "stargazer/StargazerConfig.h"
#define foreach BOOST_FOREACH

using namespace stargazer_ros_tool;

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

LandmarkCalibratorInterface::LandmarkCalibratorInterface(ros::NodeHandle node_handle,
                                                         ros::NodeHandle private_node_handle)
        : params_{LandmarkCalibratorInterfaceParameters::getInstance()} {

    // Set parameters
    params_.fromNodeHandle(private_node_handle);
    load_data();

    // Init logging for ceres
    google::InitGoogleLogging("LandmarkCalibrator");
    FLAGS_logtostderr = 1;

    // Optimize
    optimize();

    // Write data
    write_data();
}

void LandmarkCalibratorInterface::synchronizerCallback(const stargazer_ros_tool::Landmarks::ConstPtr& lm_msg,
                                                       const geometry_msgs::PoseStamped::ConstPtr& pose_msg) {

    std::vector<stargazer::Landmark> converted_landmarks;
    converted_landmarks.reserve(lm_msg->landmarks.size());
    for (auto& lm : lm_msg->landmarks) {
        converted_landmarks.push_back(convert2Landmark(lm));
    }
    observed_landmarks.push_back(converted_landmarks);
    observed_poses.push_back(
        {pose_msg->pose.position.x, pose_msg->pose.position.y, tf::getYaw(pose_msg->pose.orientation)});
}

void LandmarkCalibratorInterface::load_data() {
    //! Read Config
    ROS_INFO_STREAM("Reading config file...");
    readConfig(params_.stargazer_cfg_file_in, bundleAdjuster.camera_intrinsics, bundleAdjuster.landmarks);

    ROS_INFO_STREAM("Reading bag file...");
    rosbag::Bag bag;
    bag.open(params_.bag_file, rosbag::bagmode::Read);

    std::vector<std::string> topics;
    topics.push_back(std::string(params_.landmark_topic));
    topics.push_back(std::string(params_.pose_topic));

    // Set up fake subscribers to capture images
    BagSubscriber<stargazer_ros_tool::Landmarks> lm_sub;
    BagSubscriber<geometry_msgs::PoseStamped> pose_sub;

    // Use time synchronizer to make sure we get properly synchronized images
    message_filters::TimeSynchronizer<stargazer_ros_tool::Landmarks, geometry_msgs::PoseStamped> sync(lm_sub, pose_sub,
                                                                                                      25);
    sync.registerCallback(boost::bind(&LandmarkCalibratorInterface::synchronizerCallback, this, _1, _2));

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    foreach (rosbag::MessageInstance const m, view) {

        if (m.isType<stargazer_ros_tool::Landmarks>()) {
            stargazer_ros_tool::Landmarks::ConstPtr lm_msg = m.instantiate<stargazer_ros_tool::Landmarks>();
            lm_sub.newMessage(lm_msg);
        } else if (m.isType<geometry_msgs::PoseStamped>()) {
            geometry_msgs::PoseStamped::ConstPtr pose_msg = m.instantiate<geometry_msgs::PoseStamped>();
            pose_sub.newMessage(pose_msg);
        }
    }

    bag.close();

    std::cout << "CameraParameters: " << bundleAdjuster.camera_intrinsics.size() << std::endl;
    std::cout << "Landmarks: " << bundleAdjuster.landmarks.size() << std::endl;
    std::cout << "Observations(Images): " << observed_landmarks.size() << std::endl;
    std::cout << "Observations(Poses): " << observed_poses.size() << std::endl;
}

void LandmarkCalibratorInterface::write_data() {
    writeConfig(params_.stargazer_cfg_file_out, bundleAdjuster.camera_intrinsics, bundleAdjuster.landmarks);
    // TODO write optimized poses?
}

void LandmarkCalibratorInterface::optimize() {
    // Start work by setting up problem
    bundleAdjuster.AddCameraPoses(observed_poses);
    bundleAdjuster.AddReprojectionResidualBlocks(observed_landmarks);
    //  bundleAdjuster.SetParametersConstant();
    bundleAdjuster.Optimize();
}
