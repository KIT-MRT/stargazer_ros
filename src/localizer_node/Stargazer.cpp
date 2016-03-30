//
// Created by bandera on 28.03.16.
//

#include "Stargazer.h"


Stargazer::Stargazer(std::string cfgfile)
    : n(ros::NodeHandle()),
      np(ros::NodeHandle("~")),
      img_trans(n),
      landmarkFinder(cfgfile) {


  // Initialize publisher
  pose_pub = np.advertise<geometry_msgs::PoseStamped>("pose", 1);

  // Initialize subscribers
  img_sub = img_trans.subscribe("/image_raw", 1, &Stargazer::imgCallback, this);

  // Go and spin
  ros::spin();
}

void Stargazer::imgCallback(const sensor_msgs::ImageConstPtr &msg) {

}