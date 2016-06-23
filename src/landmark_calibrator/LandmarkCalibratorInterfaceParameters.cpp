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

#include "LandmarkCalibratorInterfaceParameters.h"

#include "../ros_utils.h"

namespace stargazer_ros {

LandmarkCalibratorInterfaceParameters& LandmarkCalibratorInterfaceParameters::getInstance() {
    static LandmarkCalibratorInterfaceParameters p;
    return p;
}

void LandmarkCalibratorInterfaceParameters::fromNodeHandle(const ros::NodeHandle& node_handle) {

    getParam(node_handle, "stargazer_cfg_file_in", stargazer_cfg_file_in);
    getParam(node_handle, "stargazer_cfg_file_out", stargazer_cfg_file_out);
    getParam(node_handle, "bag_file", bag_file);
    getParam(node_handle, "landmark_topic", landmark_topic);
    getParam(node_handle, "pose_topic", pose_topic);
}

LandmarkCalibratorInterfaceParameters::LandmarkCalibratorInterfaceParameters() {
}

} // namespace stargazer_ros
