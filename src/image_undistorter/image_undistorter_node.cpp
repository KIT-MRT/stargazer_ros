#include "ImageUndistorterInterface.h"

int main(int argc, char **argv) {

  ros::init(argc, argv, "image_undistorter_node");

  stargazer_ros_tool::ImageUndistorterInterface interface(ros::NodeHandle(),
                                                          ros::NodeHandle("~"));

  ros::spin();
  return EXIT_SUCCESS;
}
