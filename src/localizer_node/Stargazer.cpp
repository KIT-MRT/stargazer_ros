//
// Created by bandera on 28.03.16.
//

#include "Stargazer.h"


Stargazer::Stargazer(std::string cfgfile)
    : n(ros::NodeHandle()),
      np(ros::NodeHandle("~")),
      img_trans(n),
      landmarkFinder(cfgfile) ,
      localizer(cfgfile) {


  // Initialize publisher
  pose_pub = np.advertise<geometry_msgs::PoseStamped>("pose", 1);

  // Initialize subscribers
  img_sub = img_trans.subscribe("/image_raw", 1, &Stargazer::imgCallback, this);

  // Go and spin
  ros::spin();
}

void Stargazer::imgCallback(const sensor_msgs::ImageConstPtr &msg) {
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

  // Find Landmarks
  cv::Mat ImageCV = cv_ptr->image;
  std::vector<Landmark> detected_landmarks = landmarkFinder.FindLandmarks(ImageCV);

  // Localize
  localizer.UpdatePose(detected_landmarks);

  // Publish tf pose
  tf::StampedTransform transform;
  pose2tf(localizer.getPose(), transform);
  transform.stamp_ = msg->header.stamp;
  transform.frame_id_ = "world";
  transform.child_frame_id_= "vehicle";
  tf_pub.sendTransform(transform);


}