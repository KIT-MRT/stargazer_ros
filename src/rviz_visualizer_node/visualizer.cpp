//
// Created by bandera on 20.03.16.
//
#include "ros/ros.h"
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"
#include "geometry_msgs/PoseArray.h"
#include "visualization_msgs/MarkerArray.h"

// Cereal includes
#include "cereal/cereal.hpp"
#include <cereal/archives/xml.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/string.hpp>
#include <cereal/types/map.hpp>
#include <cereal/types/array.hpp>

#include <fstream>

#include "stargazer/StargazerConfig.h"

#include "ceres/rotation.h"


geometry_msgs::Pose toGeomPose(std::array<double, (int) POSE::N_PARAMS> pose_in) {
  geometry_msgs::Pose pose_out;
  pose_out.position.x = pose_in[(int) POSE::X];
  pose_out.position.y = pose_in[(int) POSE::Y];
  pose_out.position.z = pose_in[(int) POSE::Z];
  std::array<double, 4> quaternion;
  std::array<double, 3> angleAxis;
  angleAxis[0] = pose_in[(int) POSE::Rx];
  angleAxis[1] = pose_in[(int) POSE::Ry];
  angleAxis[2] = pose_in[(int) POSE::Rz]+M_PI/2; // Wegen verdrehtem Kamerasystem auf dem Auto (x-Achse nach rechts)
  ceres::AngleAxisToQuaternion(angleAxis.data(), quaternion.data());
  pose_out.orientation.w = quaternion[0];
  pose_out.orientation.x = quaternion[1];
  pose_out.orientation.y = quaternion[2];
  pose_out.orientation.z = quaternion[3];
  return pose_out;
}

void pose2tf(pose_t pose_in, tf::StampedTransform &transform) {
  transform.setOrigin(tf::Vector3(pose_in[(int) POSE::X],
                                  pose_in[(int) POSE::Y],
                                  pose_in[(int) POSE::Z]));
  double quaternion[4];
  double angleAxis[3];
  angleAxis[0] = pose_in[(int) POSE::Rx];
  angleAxis[1] = pose_in[(int) POSE::Ry];
  angleAxis[2] = pose_in[(int) POSE::Rz];
  ceres::AngleAxisToQuaternion(&angleAxis[0], &quaternion[0]);
  tf::Quaternion q;
  q.setW(quaternion[0]);
  q.setX(quaternion[1]);
  q.setY(quaternion[2]);
  q.setZ(quaternion[3]);
  transform.setRotation(q);
  return;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "static_landmark_visualizer");
  ros::NodeHandle n, np("~");

  /* Read in data */
  landmark_map_t landmarks;
  std::vector<pose_t> camera_poses;
  camera_params_t camera_intrinsics;

  std::string stargazer_cfg_file, cereal_state_file;
  if (!np.getParam("stargazer_cfg_file", stargazer_cfg_file)
      || !np.getParam("cereal_state_file", cereal_state_file)) {
    ROS_ERROR_STREAM("A parameter is missing. Have you specified all input files?. Exiting...");
    ros::shutdown();
  };
  if (!readConfig(stargazer_cfg_file,camera_intrinsics,landmarks))
    throw std::runtime_error("Could not read stargazer cfg file");
  {
    // Open and read within sub env, so that file gets closed again directly
    std::ifstream file(cereal_state_file);
    cereal::XMLInputArchive iarchive(file);
    iarchive(camera_poses);
    std::cout << "Read in " << camera_poses.size() << " camera poses." << std::endl;
  }
  ros::Publisher lm_pub = n.advertise<visualization_msgs::MarkerArray>("/landmarks", 10);
  ros::Publisher path_pub = n.advertise<geometry_msgs::PoseArray>("/path", 10);
  tf::TransformBroadcaster transformBroadcaster;

  ros::Rate r(1);
  while (ros::ok()) {
    ros::Time timestamp = ros::Time::now();

    geometry_msgs::PoseArray poseArray;
    poseArray.header.stamp = timestamp;
    poseArray.header.frame_id = "world";
    for (auto &el : camera_poses) {
      poseArray.poses.push_back(toGeomPose(el));
    }

    visualization_msgs::MarkerArray lm_msg;
    for (auto &el : landmarks) {
      std::string frame_id = "lm" + std::to_string(el.first);

      // TF
      tf::StampedTransform transform;
      transform.stamp_ = timestamp;
      transform.frame_id_ = "world";
      transform.child_frame_id_ = frame_id;
      pose2tf(el.second.pose, transform);
      transformBroadcaster.sendTransform(transform);

      // Landmark
      visualization_msgs::Marker marker;
      marker.header.stamp = timestamp;
      marker.lifetime = ros::Duration();
      marker.header.frame_id = frame_id;
      marker.id = el.first;
      marker.action = visualization_msgs::Marker::ADD;
      marker.ns = "Landmarks";
      marker.pose.position.x = Landmark::kGridDistance * (Landmark::kGridCount / 2. - 1. / 2.);
      marker.pose.position.y = Landmark::kGridDistance * (Landmark::kGridCount / 2. - 1. / 2.);
      marker.pose.position.z = -Landmark::kGridDistance / 2.;
      marker.pose.orientation.w = 1;
      marker.type = visualization_msgs::Marker::CUBE;
      marker.scale.x = Landmark::kGridDistance * (Landmark::kGridCount);
      marker.scale.y = Landmark::kGridDistance * (Landmark::kGridCount);
      marker.scale.z = 0.02;
      marker.color.r = 1.0f;
      marker.color.g = 1.0f;
      marker.color.b = 1.0f;
      marker.color.a = 1.0;
      lm_msg.markers.push_back(marker);

      // LEDS
      Landmark lm = Landmark(el.first);
      visualization_msgs::Marker led_marker;
      led_marker.header.stamp = timestamp;
      led_marker.lifetime = ros::Duration();
      led_marker.header.frame_id = frame_id;
      led_marker.id = el.first;
      led_marker.action = visualization_msgs::Marker::ADD;
      led_marker.type = visualization_msgs::Marker::SPHERE_LIST;
      led_marker.scale.x = 0.08;
      led_marker.scale.y = 0.08;
      led_marker.scale.z = 0.08;
      led_marker.color.r = 1.0f;
      led_marker.color.g = 1.0f;
      led_marker.color.b = 0.0f;
      led_marker.color.a = 1.0;
      for (auto &led : lm.points) {
        geometry_msgs::Point pt;
        pt.x = std::get<(int) POINT::X>(led);
        pt.y = std::get<(int) POINT::Y>(led);
        pt.z = 0;
        led_marker.points.push_back(pt);
      }
      lm_msg.markers.push_back(led_marker);

      // Text
      visualization_msgs::Marker text_marker;
      text_marker.header.stamp = timestamp;
      text_marker.lifetime = ros::Duration();
      text_marker.header.frame_id = frame_id;
      text_marker.id = 1000 * el.first;
      text_marker.text = std::to_string(el.first);
      text_marker.action = visualization_msgs::Marker::ADD;
      text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      text_marker.scale.z = 0.1;
      text_marker.color.r = 1.0f;
      text_marker.color.g = 1.0f;
      text_marker.color.b = 0.0f;
      text_marker.color.a = 1.0;
      text_marker.pose.position.x = Landmark::kGridDistance * (Landmark::kGridCount / 2. - 1. / 2.);
      text_marker.pose.position.y = Landmark::kGridDistance * (Landmark::kGridCount / 2. - 1. / 2.);
      text_marker.pose.position.z = -Landmark::kGridDistance * 2.;
      text_marker.pose.orientation.w = 1;
      lm_msg.markers.push_back(text_marker);
    }

    lm_pub.publish(lm_msg);
    path_pub.publish(poseArray);

    ros::spinOnce();
    r.sleep();
  }

  return 0;
}