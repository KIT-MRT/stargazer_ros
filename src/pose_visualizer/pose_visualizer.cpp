//
// Created by bandera on 20.03.16.
//
#include <boost/foreach.hpp>
#include <geometry_msgs/PoseArray.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include "PoseVisualizerParameters.h"
#include "../StargazerConversionMethods.h"

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

    geometry_msgs::PoseArray pose_array, pose_opt_array;

    rosbag::Bag bag(params.bag_file, rosbag::bagmode::Read);

    std::vector<std::string> topics;
    topics.push_back(std::string(params.pose_topic));
    topics.push_back(std::string(params.pose_optimized_topic));
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    foreach (rosbag::MessageInstance const m, view) {
        geometry_msgs::PoseStamped::ConstPtr pose_msg = m.instantiate<geometry_msgs::PoseStamped>();

        if (m.getTopic() == params.pose_topic) {
            pose_array.header = pose_msg->header;
            pose_array.header.frame_id = params.world_frame;
            pose_array.poses.push_back(pose_msg->pose);
        } else if (m.getTopic() == params.pose_optimized_topic) {
            pose_opt_array.header = pose_msg->header;
            pose_opt_array.header.frame_id = params.world_frame;
            pose_opt_array.poses.push_back(pose_msg->pose);
        }
    }
    bag.close();

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