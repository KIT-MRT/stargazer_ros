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

#pragma once
#include <ros/ros.h>

namespace stargazer_ros {

/**
 * Show summary about node containing name, namespace,
 * subscribed and advertised topics.
 */
inline void showNodeInfo() {

    using namespace ros::this_node;

    std::vector<std::string> subscribed_topics, advertised_topics;
    getSubscribedTopics(subscribed_topics);
    getAdvertisedTopics(advertised_topics);

    std::ostringstream msg_subscr, msg_advert;
    for (auto const& t : subscribed_topics) {
        msg_subscr << t << std::endl;
    }
    for (auto const& t : advertised_topics) {
        msg_advert << t << std::endl;
    }

    ROS_INFO_STREAM("Started '" << getName() << "' in namespace '" << getNamespace() << "'." << std::endl
                                << "Subscribed topics: " << std::endl
                                << msg_subscr.str() << "Advertised topics: " << std::endl
                                << msg_advert.str());
}

template <typename T>
inline void getParam(const ros::NodeHandle& node_handle, const std::string key, T& val) {

    if (!node_handle.getParam(key, val)) {
        ROS_ERROR_STREAM("Undefined parameter '" << key << "'.");
        std::exit(EXIT_FAILURE);
    }
}

} // namespace stargazer_ros
