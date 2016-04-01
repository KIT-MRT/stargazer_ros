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

  if (argc < 4) {
    // We print argv[0] assuming it is the program name
    std::cout << "Usage: " << argv[0] << " <config file> <landmark_observations.xml> <pose_observations.xml>"
        << std::endl;
    std::cin.get();
    exit(0);
  }
  /* Get params */
  std::string stargazer_cfg_file(argv[1]);
  std::string cereal_lm_file(argv[2]);
  std::string cereal_state_file(argv[3]);

  // Init logging for ceres
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = 1;

  BundleAdjuster bundleAdjuster;

  //! Read Config
  std::cout << "reading config files..." << std::endl;
  assert(readConfig(stargazer_cfg_file, bundleAdjuster.camera_intrinsics, bundleAdjuster.landmarks));

  // Read in measurements
  std::cout << "reading measurements files..." << std::endl;
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
  std::cout << "Landmarks: " << bundleAdjuster.landmarks.size() << std::endl;
  std::cout << "Observations(Images): " << measurements_converted.size() << std::endl;
  std::cout << "Observations(Poses): " << observed_poses.size() << std::endl;



  // Start work by setting up problem
  bundleAdjuster.AddCameraPoses(observed_poses);
  bundleAdjuster.AddReprojectionResidualBlocks(measurements_converted);
//  bundleAdjuster.SetParametersConstant();
  bundleAdjuster.Optimize();




  // Save data.
  // Get current directory
  char cCurrentPath[FILENAME_MAX];
  if (!getcwd(cCurrentPath, sizeof(cCurrentPath)))
    return errno;
  cCurrentPath[sizeof(cCurrentPath) - 1] = '\0'; /* not really required */
  std::string output_dir(cCurrentPath);
  output_dir += "/";

  std::cout << "Saving config files to " << output_dir << std::endl;
  assert(writeConfig(output_dir + "stargazer_optimized.yaml",
                     bundleAdjuster.camera_intrinsics,
                     bundleAdjuster.landmarks));
  {
    std::ofstream file(output_dir + "poses_optimized.xml");
    cereal::XMLOutputArchive oarchive(file); // Create an output archive
    oarchive(bundleAdjuster.camera_poses);
  }
  return 0;
}