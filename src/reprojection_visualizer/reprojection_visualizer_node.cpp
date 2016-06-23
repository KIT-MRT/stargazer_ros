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

int main(int argc, char** argv) {

    ros::init(argc, argv, "reprojection_visualizer_node");

    stargazer_ros::ReprojectionVisualizer reprojectionVisualizer(ros::NodeHandle(), ros::NodeHandle("~"));

    //    ros::spin(); // We don't need that here
    return EXIT_SUCCESS;
}
