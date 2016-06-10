//
// Created by bandera on 10.06.16.
//

#include "LandmarkCalibratorInterface.h"

// ROS includes
#include <rosbag/bag.h>
#include <rosbag/view.h>

// ROS messages
#include <geometry_msgs/PoseStamped.h>
#include "stargazer_ros_tool/Landmarks.h"

// Local Helpers
#include <boost/foreach.hpp>
#include <tf/transform_datatypes.h>
#include "stargazer/StargazerConfig.h"
#define foreach BOOST_FOREACH

using namespace stargazer_ros_tool;

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

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    foreach (rosbag::MessageInstance const m, view) {

        if (m.isType<stargazer_ros_tool::Landmarks>()) {
            stargazer_ros_tool::Landmarks::ConstPtr lm_msg = m.instantiate<stargazer_ros_tool::Landmarks>();
            std::vector<stargazer::Landmark> converted_landmarks;
            converted_landmarks.reserve(lm_msg->landmarks.size());
            for (auto& lm : lm_msg->landmarks) {
                converted_landmarks.push_back(convert2Landmark(lm));
            }
            observed_landmarks.push_back(converted_landmarks);
        } else if (m.isType<geometry_msgs::PoseStamped>()) {
            geometry_msgs::PoseStamped::ConstPtr pose_msg = m.instantiate<geometry_msgs::PoseStamped>();
            observed_poses.push_back(
                {pose_msg->pose.position.x, pose_msg->pose.position.y, tf::getYaw(pose_msg->pose.orientation)});
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

stargazer::Landmark LandmarkCalibratorInterface::convert2Landmark(const stargazer_ros_tool::Landmark& lm_in) {
    stargazer::Landmark lm_out(lm_in.id);
    lm_out.points.clear();

    for (auto& el : lm_in.corner_points) {
        stargazer::Point pt = {el.u, el.v, 0};
        lm_out.points.push_back(pt);
    }
    for (auto& el : lm_in.id_points) {
        stargazer::Point pt = {el.u, el.v, 0};
        lm_out.points.push_back(pt);
    }

    return lm_out;
};
