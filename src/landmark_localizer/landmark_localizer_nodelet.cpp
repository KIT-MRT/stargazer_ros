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

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "LandmarkLocalizerInterface.h"

namespace stargazer_ros {

class LandmarkLocalizerInterfaceNodelet : public nodelet::Nodelet {

    virtual void onInit();
    std::unique_ptr<LandmarkLocalizerInterface> m_;
};

void LandmarkLocalizerInterfaceNodelet::onInit() {
    m_ = std::make_unique<LandmarkLocalizerInterface>(getNodeHandle(), getPrivateNodeHandle());
}
} // namespace image_undistorter

PLUGINLIB_DECLARE_CLASS(stargazer_ros, LandmarkLocalizerInterfaceNodelet,
                        stargazer_ros::LandmarkLocalizerInterfaceNodelet, nodelet::Nodelet);
