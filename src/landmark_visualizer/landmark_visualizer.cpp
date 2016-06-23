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
#include "LandmarkVisualizerParameters.h"
#include "../StargazerConversionMethods.h"
#include "../ros_utils.h"
#include "ceres/rotation.h"
#include "ros/ros.h"
#include "stargazer/StargazerConfig.h"
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"
#include "visualization_msgs/MarkerArray.h"

using namespace stargazer;
using namespace stargazer_ros;

int main(int argc, char** argv) {
    ros::init(argc, argv, "static_landmark_visualizer");
    ros::NodeHandle n, np("~");

    /* Read in data */
    landmark_map_t landmarks;
    camera_params_t camera_intrinsics;
    LandmarkVisualizerParameters& params = LandmarkVisualizerParameters::getInstance();
    params.fromNodeHandle(np);

    readConfig(params.stargazer_config, camera_intrinsics, landmarks);

    ros::Publisher lm_pub = n.advertise<visualization_msgs::MarkerArray>(params.landmark_topic, 1);
    tf::TransformBroadcaster transformBroadcaster;

    // Prepare data
    visualization_msgs::MarkerArray lm_msg;
    std::vector<tf::StampedTransform> transforms;

    for (auto& el : landmarks) {
        stargazer::Landmark& lm = el.second;
        std::string frame_id = "lm" + std::to_string(lm.id);

        // TF
        tf::StampedTransform transform;
        transform.frame_id_ = "world";
        transform.child_frame_id_ = frame_id;
        pose2tf(lm.pose, transform);
        transforms.push_back(transform);

        // Landmark
        visualization_msgs::Marker marker;
        marker.lifetime = ros::Duration();
        marker.header.frame_id = frame_id;
        marker.id = lm.id;
        marker.action = visualization_msgs::Marker::ADD;
        marker.ns = "Landmarks";
        marker.pose.position.x = stargazer::Landmark::kGridDistance * (stargazer::Landmark::kGridCount / 2. - 1. / 2.);
        marker.pose.position.y = stargazer::Landmark::kGridDistance * (stargazer::Landmark::kGridCount / 2. - 1. / 2.);
        marker.pose.position.z = -stargazer::Landmark::kGridDistance / 2.;
        marker.pose.orientation.w = 1;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.scale.x = stargazer::Landmark::kGridDistance * (stargazer::Landmark::kGridCount);
        marker.scale.y = stargazer::Landmark::kGridDistance * (stargazer::Landmark::kGridCount);
        marker.scale.z = 0.02;
        marker.color.r = 1.0f;
        marker.color.g = 1.0f;
        marker.color.b = 1.0f;
        marker.color.a = 1.0;
        lm_msg.markers.push_back(marker);

        // LEDS
        visualization_msgs::Marker led_marker;
        led_marker.lifetime = ros::Duration();
        led_marker.header.frame_id = frame_id;
        led_marker.id = lm.id;
        led_marker.action = visualization_msgs::Marker::ADD;
        led_marker.type = visualization_msgs::Marker::SPHERE_LIST;
        led_marker.scale.x = 0.08;
        led_marker.scale.y = 0.08;
        led_marker.scale.z = 0.08;
        led_marker.color.r = 1.0f;
        led_marker.color.g = 1.0f;
        led_marker.color.b = 0.0f;
        led_marker.color.a = 1.0;
        for (auto& led : lm.points) {
            geometry_msgs::Point pt;
            pt.x = std::get<(int)POINT::X>(led);
            pt.y = std::get<(int)POINT::Y>(led);
            pt.z = 0;
            led_marker.points.push_back(pt);
        }
        lm_msg.markers.push_back(led_marker);

        // Text
        visualization_msgs::Marker text_marker;
        text_marker.lifetime = ros::Duration();
        text_marker.header.frame_id = frame_id;
        text_marker.id = 1000 * lm.id;
        text_marker.text = std::to_string(lm.id);
        text_marker.action = visualization_msgs::Marker::ADD;
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.scale.z = 0.1;
        text_marker.color.r = 1.0f;
        text_marker.color.g = 1.0f;
        text_marker.color.b = 0.0f;
        text_marker.color.a = 1.0;
        text_marker.pose.position.x =
            stargazer::Landmark::kGridDistance * (stargazer::Landmark::kGridCount / 2. - 1. / 2.);
        text_marker.pose.position.y =
            stargazer::Landmark::kGridDistance * (stargazer::Landmark::kGridCount / 2. - 1. / 2.);
        text_marker.pose.position.z = -stargazer::Landmark::kGridDistance * 2.;
        text_marker.pose.orientation.w = 1;
        lm_msg.markers.push_back(text_marker);
    }

    showNodeInfo();

    // Start loop
    ros::Rate r(params.rate);
    while (ros::ok()) {
        ros::Time timestamp = ros::Time::now();

        for (auto& transform : transforms) {
            transform.stamp_ = timestamp;
            transformBroadcaster.sendTransform(transform);
        }

        for (auto& marker : lm_msg.markers) {
            marker.header.stamp = timestamp;
        }
        lm_pub.publish(lm_msg);

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}