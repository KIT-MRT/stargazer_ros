#include "LandmarkCalibratorInterface.h"

int main(int argc, char** argv) {

    ros::init(argc, argv, "landmark_calibrator_node");

    stargazer_ros_tool::LandmarkCalibratorInterface interface(ros::NodeHandle(), ros::NodeHandle("~"));

    ros::spin();
    return EXIT_SUCCESS;
}
