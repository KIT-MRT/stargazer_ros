#pragma once

#include "stargazer/StargazerTypes.h"


typedef struct {
  int nID;
  std::vector<std::vector<double>> voCorners;
  std::vector<std::vector<double>> voIDPoints;
} StarLandmark;

Landmark convert2Landmark(StarLandmark lm_in) {
  Landmark lm_out(lm_in.nID);
  lm_out.points.clear();

  for (auto &el:lm_in.voCorners)
    lm_out.points.push_back(std::make_tuple(0, el[0], el[1]));
  for (auto &el:lm_in.voIDPoints)
    lm_out.points.push_back(std::make_tuple(0, el[0], el[1]));

  return lm_out;
};

/////////////////////////////////////
/// Serialization
/////////////////////////////////////
namespace cereal {
template<class Archive>
inline
void serialize(Archive &ar, StarLandmark &lm) {
  ar(lm.nID, lm.voCorners, lm.voIDPoints);
}
} // namespace cereal
