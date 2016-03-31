//
// Created by bandera on 23.03.16.
//
// Cereal includes
#include "cereal/cereal.hpp"
#include <cereal/archives/xml.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/string.hpp>
#include <cereal/types/map.hpp>
#include <cereal/types/array.hpp>

#include <fstream>

#include "stargazer/StargazerTypes.h"
#include "stargazer/StargazerConfig.h"
#include "stargazer/internal/CostFunction.h"

#include "../bundle_adjuster_node/StarLandmark.h"

#include <opencv/highgui.h>


int main(int argc, char **argv) {

  /* Read in data */
  landmark_map_t landmark_poses;
  std::vector<pose_t> camera_poses;
  std::vector<std::vector<StarLandmark >> measurements_raw;
  std::vector<std::vector<Landmark >> measurements;
  camera_params_t camera_intrinsics;

  assert(readConfig("/home/bandera/repos/kitcar/bundle_adjuster/src/stargazer_ros_tool/res/stargazer_optimized.yaml",
                    camera_intrinsics,
                    landmark_poses));

  {
    // Open and read within sub env, so that file gets closed again directly
    std::ifstream file("/home/bandera/repos/kitcar/bundle_adjuster/src/stargazer_ros_tool/res/poses_optimized.xml");
    cereal::XMLInputArchive iarchive(file);
    iarchive(camera_poses);
    std::cout << "Read in " << camera_poses.size() << " camera poses." << std::endl;
    assert(!camera_poses.empty());
  }
  {
    // Open and read within sub env, so that file gets closed again directly
    std::ifstream
        file2("/home/bandera/repos/kitcar/bundle_adjuster/src/stargazer_ros_tool/res/observed_landmarks.xml");
    cereal::XMLInputArchive iarchive(file2);
    std::string timestamp;
    iarchive(measurements_raw);
    for (auto &obs:measurements_raw) {
      std::vector<Landmark> tmp;
      for (auto &lm:obs)
        tmp.push_back(convert2Landmark(lm));
      measurements.push_back(tmp);
    }
    std::cout << "Read in " << measurements.size() << " measurements." << std::endl;
    assert(camera_poses.size() == measurements.size());
  }


  for (int j = 0; j < measurements.size(); j++) {

    auto shot = measurements[j];
    auto camera_pose = camera_poses[j];
    cv::Mat img = cv::Mat::zeros(1048, 1363, CV_8UC3); ///@todo read those in from somewhere!

    for (int k = 0; k < shot.size(); k++) {
      Landmark lm_obs = shot[k];
      Landmark lm_real(lm_obs.id);
      auto lm_pose = landmark_poses[lm_real.id];

      for (int i = 0; i < lm_obs.points.size(); i++) {

        // Transform landmark point to camera
        double u_marker, v_marker;
        double x_marker = std::get<(int) POINT::X>(lm_real.points[i]);
        double y_marker = std::get<(int) POINT::Y>(lm_real.points[i]);
        transformPoint<double>(&x_marker,
                               &y_marker,
                               lm_pose.data(),
                               camera_pose.data(),
                               camera_intrinsics.data(),
                               &u_marker, &v_marker);

        auto pt_real = cvPoint(u_marker, v_marker);
        auto pt_obs = cvPoint(std::get<(int) POINT::X>(lm_obs.points[i]), std::get<(int) POINT::Y>(lm_obs.points[i]));
//        circle(img, pt_real, 3, cv::Scalar(0, 0, 255), 2);   //Red
//        circle(img, pt_obs, 3, cv::Scalar(255, 0, 0), 2);   //Blue

//        if (i == 0) { // first point
        std::stringstream out1;
//          out1 << lm_real.id;
        out1 << i;
        std::string txt;
        txt = out1.str();
        cv::Point tmppt = pt_real;
        tmppt.x += 10;
        tmppt.y += 10;
        putText(img, txt, pt_real, 2, 0.4, cvScalar(0, 0, 255));
        tmppt = pt_obs;
        tmppt.x += 10;
        tmppt.y += 10;
        putText(img, txt, pt_obs, 2, 0.4, cvScalar(255, 0, 0));
//        }
      }
    }
    try {
      cv::namedWindow("Image window", CV_WINDOW_NORMAL);
      cvSetWindowProperty("Image window", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
      cv::imshow("Image window", img);
      cv::waitKey(100);
    }
    catch (cv::Exception &cv_ex) {
      std::cerr << "cv exception: " << cv_ex.what() << std::endl;
      return 0;
    }
  }
  return 0;
}
