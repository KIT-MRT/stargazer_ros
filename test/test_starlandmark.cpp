//google test docs
//wiki page: https://code.google.com/p/googletest/w/list
//primer: https://code.google.com/p/googletest/wiki/V1_7_Primer
//FAQ: https://code.google.com/p/googletest/wiki/FAQ
//advanced guide: https://code.google.com/p/googletest/wiki/V1_7_AdvancedGuide
//samples: https://code.google.com/p/googletest/wiki/V1_7_Samples
//
//List of some basic tests fuctions:
//Fatal assertion                      Nonfatal assertion                   Verifies / Description
//-------------------------------------------------------------------------------------------------------------------------------------------------------
//ASSERT_EQ(expected, actual);         EXPECT_EQ(expected, actual);         expected == actual
//ASSERT_NE(val1, val2);               EXPECT_NE(val1, val2);               val1 != val2
//ASSERT_LT(val1, val2);               EXPECT_LT(val1, val2);               val1 < val2
//ASSERT_LE(val1, val2);               EXPECT_LE(val1, val2);               val1 <= val2
//ASSERT_GT(val1, val2);               EXPECT_GT(val1, val2);               val1 > val2
//ASSERT_GE(val1, val2);               EXPECT_GE(val1, val2);               val1 >= val2
//
//ASSERT_FLOAT_EQ(expected, actual);   EXPECT_FLOAT_EQ(expected, actual);   the two float values are almost equal (4 ULPs)
//ASSERT_DOUBLE_EQ(expected, actual);  EXPECT_DOUBLE_EQ(expected, actual);  the two double values are almost equal (4 ULPs)
//ASSERT_NEAR(val1, val2, abs_error);  EXPECT_NEAR(val1, val2, abs_error);  the difference between val1 and val2 doesn't exceed the given absolute error
//
//Note: more information about ULPs can be found here: http://www.cygnus-software.com/papers/comparingfloats/comparingfloats.htm
//
//Example of two unit test:
//TEST(Math, Add) {
//    ASSERT_EQ(10, 5+ 5);
//}
//
//TEST(Math, Float) {
//	  ASSERT_FLOAT_EQ((10.0f + 2.0f) * 3.0f, 10.0f * 3.0f + 2.0f * 3.0f)
//}
//=======================================================================================================================================================
#include "gtest/gtest.h"
#include "../src/bundle_adjuster_node/StarLandmark.h"


TEST(StarLandmark, ConvertStarLandmarkToLandmark) {
  StarLandmark starLandmark;
  starLandmark.nID = 8464;
  starLandmark.voCorners = {{195, 606, 0}, {218, 607, 0}, {219, 584, 0}};
  starLandmark.voIDPoints = {{195, 598, 0}, {195, 590, 0}, {203, 583, 0}};
  Landmark lm = convert2Landmark(starLandmark);
  ASSERT_EQ(8464, lm.id);
  ASSERT_EQ(starLandmark.voCorners[0][0], std::get<(int) POINT::X>(lm.points[0]));
  ASSERT_EQ(starLandmark.voCorners[0][1], std::get<(int) POINT::Y>(lm.points[0]));
  ASSERT_EQ(starLandmark.voIDPoints[1][0], std::get<(int) POINT::X>(lm.points[4]));
  ASSERT_EQ(starLandmark.voIDPoints[1][1], std::get<(int) POINT::Y>(lm.points[4]));
}