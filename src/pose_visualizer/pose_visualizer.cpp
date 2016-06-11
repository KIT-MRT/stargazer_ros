//
// Created by bandera on 20.03.16.
//
#include <boost/foreach.hpp>
#include <geometry_msgs/PoseArray.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf/transform_listener.h>
#include <utils_ros/ros_console.hpp>
#include "PoseVisualizerParameters.h"
#include "../StargazerConversionMethods.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#define foreach BOOST_FOREACH

using namespace stargazer;
using namespace stargazer_ros_tool;

int main(int argc, char** argv) {
    ros::init(argc, argv, "pose_visualizer");
    ros::NodeHandle n, np("~");

    PoseVisualizerParameters& params = PoseVisualizerParameters::getInstance();
    params.fromNodeHandle(np);

    ros::Publisher path_pub = n.advertise<geometry_msgs::PoseArray>(params.pose_pub_topic, 1);
    ros::Publisher path_opt_pub = n.advertise<geometry_msgs::PoseArray>(params.pose_opt_pub_topic, 1);

    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener tfl(buffer);
    bool transform_found = false;
    geometry_msgs::TransformStamped cam2robotTransform;
    while (!transform_found) {
        try {
            cam2robotTransform =
                buffer.lookupTransform(params.robot_frame, params.camera_frame, ros::Time(0), ros::Duration(1.0));
            transform_found = true;
        } catch (tf2::TransformException& ex) {
            ROS_WARN_STREAM("Transform " << params.camera_frame << " -> " << params.robot_frame << " not found yet!");
        }
    }

    geometry_msgs::PoseArray pose_array, pose_opt_array;

    rosbag::Bag bag(params.bag_file, rosbag::bagmode::Read);

    std::vector<std::string> topics;
    topics.push_back(std::string(params.pose_topic));
    topics.push_back(std::string(params.pose_optimized_topic));
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    foreach (rosbag::MessageInstance const m, view) {
        geometry_msgs::PoseStamped::ConstPtr camera_pose = m.instantiate<geometry_msgs::PoseStamped>();
        geometry_msgs::PoseStamped robotPose = *camera_pose;
//        tf::Quaternion q1, q2, q3;
//        q1.setW(camera_pose->pose.orientation.w);
//        q1.setX(camera_pose->pose.orientation.x);
//        q1.setY(camera_pose->pose.orientation.y);
//        q1.setZ(camera_pose->pose.orientation.z);
//        q2.setW(cam2robotTransform.transform.rotation.w);
//        q2.setX(cam2robotTransform.transform.rotation.x);
//        q2.setY(cam2robotTransform.transform.rotation.y);
//        q2.setZ(cam2robotTransform.transform.rotation.z);
//        q3 = q1 - q2;
//        q3.normalize();
//        robotPose.pose.orientation.x = q3.getX();
//        robotPose.pose.orientation.y = q3.getY();
//        robotPose.pose.orientation.z = q3.getZ();
//        robotPose.pose.orientation.w = q3.getW();
        //        tf2::doTransform(*camera_pose, robotPose, cam2robotTransform);
        if (m.getTopic() == params.pose_topic) {
            pose_array.poses.push_back(robotPose.pose);
        } else if (m.getTopic() == params.pose_optimized_topic) {
            pose_opt_array.poses.push_back(robotPose.pose);
        }
    }
    bag.close();
    pose_array.header.frame_id = params.map_frame;
    pose_opt_array.header.frame_id = params.map_frame;

    utils_ros::showNodeInfo();

    ros::Rate r(params.rate);
    while (ros::ok()) {
        ros::Time timestamp = ros::Time::now();

        pose_array.header.stamp = timestamp;
        pose_opt_array.header.stamp = timestamp;

        path_pub.publish(pose_array);
        path_opt_pub.publish(pose_opt_array);

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}