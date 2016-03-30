//
// Created by bandera on 28.03.16.
//

// ROS includes
#include <ros/ros.h>
#include "Stargazer.h"


int main(int argc, char **argv) {
  ros::init(argc, argv, "stargazer");

  std::string cfgfile;
  ros::NodeHandle n;
  if(!n.getParam("stargazer_cfg_file", cfgfile)){
    ROS_FATAL("stargazer_cfg_file not specified. Exiting...");
    ros::shutdown();
  }
  Stargazer stargazer(cfgfile);

  return 0;
}
