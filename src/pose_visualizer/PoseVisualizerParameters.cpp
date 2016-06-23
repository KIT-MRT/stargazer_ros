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

#include "PoseVisualizerParameters.h"

#include "../ros_utils.h"

namespace stargazer_ros {

PoseVisualizerParameters& PoseVisualizerParameters::getInstance() {
    static PoseVisualizerParameters p;
    return p;
}

void PoseVisualizerParameters::fromNodeHandle(const ros::NodeHandle& node_handle) {

    getParam(node_handle, "bag_file", bag_file);
    getParam(node_handle, "stargazer_config", stargazer_config);
    getParam(node_handle, "map_frame", map_frame);
    getParam(node_handle, "robot_frame", robot_frame);
    getParam(node_handle, "camera_frame", camera_frame);
    getParam(node_handle, "landmark_topic", landmark_topic);
    getParam(node_handle, "pose_pub_topic", pose_pub_topic);
    getParam(node_handle, "rate", rate);
}

PoseVisualizerParameters::PoseVisualizerParameters() {
}

} // namespace stargazer_ros
