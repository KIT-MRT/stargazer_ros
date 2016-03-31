// ROS includes
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/tf.h>

// Bundle Adjuster
#include "stargazer/BundleAdjuster.h"

// Cereal includes
#include "cereal/cereal.hpp"
#include <cereal/archives/xml.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/string.hpp>
#include <cereal/types/map.hpp>
#include <cereal/types/array.hpp>

// Local Helpers
#include "stargazer/StargazerConfig.h"
#include "StarLandmark.h"


int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = 1;
//  FLAGS_v = 5;
  ros::init(argc, argv, "bundle_adjuster");
  ros::NodeHandle node_handle("bundle_adjuster");

  BundleAdjuster bundleAdjuster;

  /* Get params */
  std::string stargazer_cfg_file, cereal_lm_file, cereal_state_file;
  if (!node_handle.getParam("stargazer_cfg_file", stargazer_cfg_file)
      || !node_handle.getParam("cereal_lm_file", cereal_lm_file)
      || !node_handle.getParam("cereal_state_file", cereal_state_file)) {
    ROS_ERROR_STREAM(
        "A parameter is missing. Have you specified 'landmark_cfg_file' and 'calibration_file' and 'cereal_file'?. Exiting...");
    ros::shutdown();
  };

  //! Read Config
  ROS_INFO("reading config files...");
  assert(readConfig(stargazer_cfg_file, bundleAdjuster.camera_intrinsics, bundleAdjuster.landmark_poses));

  // Read in measurements
  ROS_INFO("bundle_adjuster reading measurements files...");
  std::vector<std::vector<StarLandmark >> measurements;
  {
    // Open and read within sub env, so that file gets closed again directly
    std::ifstream file2(cereal_lm_file);
    cereal::XMLInputArchive iarchive(file2);
    std::string timestamp;
    iarchive(measurements);
  }
  std::vector<std::vector<Landmark >> measurements_converted;
  for (auto &obs:measurements) {
    std::vector<Landmark> tmp;
    for (auto &lm:obs)
      tmp.push_back(convert2Landmark(lm));
    measurements_converted.push_back(tmp);
  }

  // Read in camera poses
  std::vector<std::array<double, 3>> observed_poses;
  {
    // Open and read within sub env, so that file gets closed again directly
    std::ifstream file2(cereal_state_file);
    cereal::XMLInputArchive iarchive(file2);
    std::string timestamp;
    iarchive(observed_poses);
  }
  std::cout << "CameraParameters: " << bundleAdjuster.camera_intrinsics.size() << std::endl;
  std::cout << "Landmarks: " << bundleAdjuster.landmark_poses.size() << std::endl;
  std::cout << "Observations(Images): " << measurements_converted.size() << std::endl;
  std::cout << "Observations(Poses): " << observed_poses.size() << std::endl;

  // Start work by setting up problem
  bundleAdjuster.AddCameraPoses(observed_poses);
  bundleAdjuster.AddReprojectionResidualBlocks(measurements_converted);
  bundleAdjuster.SetParametersConstant();
  bundleAdjuster.Optimize();

  // Save data.
  std::string output_dir = ros::package::getPath("stargazer_ros_tool");
  if (output_dir.empty()) {
    output_dir = "/home/bandera/Desktop";
//    bundleAdjuster.showSetup();
//    ROS_ERROR("Not saving data, because output_dir is empty.");
//    return 1;
  }
  output_dir += "/res/";

  ROS_INFO_STREAM("Saving config files to " << output_dir);
  assert(writeConfig(output_dir + "stargazer_optimized.yaml",
                     bundleAdjuster.camera_intrinsics,
                     bundleAdjuster.landmark_poses));
  {
    std::ofstream file(output_dir + "poses_optimized.xml");
    cereal::XMLOutputArchive oarchive(file); // Create an output archive
    oarchive(bundleAdjuster.camera_poses);
  }
  return 0;
}