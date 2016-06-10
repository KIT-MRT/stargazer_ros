//
// Created by bandera on 10.06.16.
//

#pragma once
#include "stargazer/StargazerImgTypes.h"
#include "stargazer/StargazerTypes.h"
#include "stargazer_ros_tool/Landmarks.h"

namespace stargazer_ros_tool {

inline stargazer::Landmark convert2Landmark(const stargazer_ros_tool::Landmark& lm_in) {

    stargazer::Landmark lm_out(lm_in.id);
    lm_out.points.clear();
    lm_out.points.reserve(lm_in.corner_points.size() + lm_in.id_points.size());

    for (auto& el : lm_in.corner_points) {
        stargazer::Point pt = {(double)el.u, (double)el.v, 0};
        lm_out.points.push_back(pt);
    }
    for (auto& el : lm_in.id_points) {
        stargazer::Point pt = {(double)el.u, (double)el.v, 0};
        lm_out.points.push_back(pt);
    }

    return lm_out;
};

inline stargazer::ImgLandmark convert2ImgLandmark(const stargazer_ros_tool::Landmark& lm_in) {

    stargazer::ImgLandmark lm_out;
    lm_out.nID = lm_in.id;

    lm_out.voCorners.reserve(lm_in.corner_points.size());
    for (auto& el : lm_in.corner_points) {
        cv::Point pt;
        pt.x = el.u;
        pt.y = el.v;
        lm_out.voCorners.push_back(pt);
    }

    lm_out.voIDPoints.reserve(lm_in.id_points.size());
    for (auto& el : lm_in.id_points) {
        cv::Point pt;
        pt.x = el.u;
        pt.y = el.v;
        lm_out.voIDPoints.push_back(pt);
    }

    return lm_out;
};

inline std::vector<stargazer::Landmark> convert2Landmarks(const stargazer_ros_tool::Landmarks& lms_in) {
    std::vector<stargazer::Landmark> lms_out;
    lms_out.reserve(lms_in.landmarks.size());

    for (auto& lm_in : lms_in.landmarks) {
        lms_out.push_back(convert2Landmark(lm_in));
    }

    return lms_out;
}

inline std::vector<stargazer::ImgLandmark> convert2ImgLandmarks(const stargazer_ros_tool::Landmarks& lms_in) {
    std::vector<stargazer::ImgLandmark> lms_out;
    lms_out.reserve(lms_in.landmarks.size());

    for (auto& lm_in : lms_in.landmarks) {
        lms_out.push_back(convert2ImgLandmark(lm_in));
    }

    return lms_out;
}

inline stargazer_ros_tool::Landmarks convert2LandmarkMsg(const std::vector<stargazer::ImgLandmark>& lm_in,
                                                         std_msgs::Header header = {}) {

    stargazer_ros_tool::Landmarks landmarksMessage;
    landmarksMessage.landmarks.reserve(lm_in.size());
    landmarksMessage.header = header;

    for (auto& lm : lm_in) {
        stargazer_ros_tool::Landmark landmark;
        landmark.header = header;
        landmark.id = lm.nID;

        for (auto& pt : lm.voCorners) {
            stargazer_ros_tool::LandmarkPoint lmpt;
            lmpt.u = static_cast<stargazer_ros_tool::LandmarkPoint::_u_type>(pt.x);
            lmpt.v = static_cast<stargazer_ros_tool::LandmarkPoint::_v_type>(pt.y);
            landmark.corner_points.push_back(lmpt);
        }

        for (auto& pt : lm.voIDPoints) {
            stargazer_ros_tool::LandmarkPoint lmpt;
            lmpt.u = static_cast<stargazer_ros_tool::LandmarkPoint::_u_type>(pt.x);
            lmpt.v = static_cast<stargazer_ros_tool::LandmarkPoint::_v_type>(pt.y);
            landmark.id_points.push_back(lmpt);
        }

        landmarksMessage.landmarks.push_back(landmark);
    }

    return landmarksMessage;
}
} // namespace stargazer_ros_tool
