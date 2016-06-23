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

#include "LandmarkFinderInterfaceParameters.h"

#include "../ros_utils.h"

namespace stargazer_ros {

LandmarkFinderInterfaceParameters& LandmarkFinderInterfaceParameters::getInstance() {
    static LandmarkFinderInterfaceParameters p;
    return p;
}

void LandmarkFinderInterfaceParameters::fromNodeHandle(const ros::NodeHandle& node_handle) {

    getParam(node_handle, "stargazer_config", stargazer_config);
    getParam(node_handle, "landmark_topic", landmark_topic);
    getParam(node_handle, "undistorted_image_topic", undistorted_image_topic);
    getParam(node_handle, "threshold", threshold);
    getParam(node_handle, "maxRadiusForPixelCluster", maxRadiusForPixelCluster);
    getParam(node_handle, "minPixelForCluster", minPixelForCluster);
    getParam(node_handle, "maxPixelForCluster", maxPixelForCluster);
    getParam(node_handle, "maxRadiusForCluster", maxRadiusForCluster);
    getParam(node_handle, "minPointsPerLandmark", minPointsPerLandmark);
    getParam(node_handle, "maxPointsPerLandmark", maxPointsPerLandmark);
    getParam(node_handle, "debug_mode", debug_mode);
}

LandmarkFinderInterfaceParameters::LandmarkFinderInterfaceParameters() {
}

} // namespace stargazer_ros
