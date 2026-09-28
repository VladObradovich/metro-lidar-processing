#include <gtest/gtest.h>

#include <cmath>

#include "metro_perception_rviz/assessment_text.hpp"

using metro_perception_interfaces::msg::ObstacleTrack;
using metro_perception_interfaces::msg::PathAssessment;
using metro_perception_rviz::assessment_text;

TEST(AssessmentText, ObstacleListsConfirmedTracksNearestFirst) {
  PathAssessment assessment;
  assessment.state = PathAssessment::OBSTACLE;
  assessment.reason = "OBSTACLE_WITH_ASSUMED_CALIBRATION";
  assessment.distance_m = 18.9;
  assessment.distance_valid = true;
  assessment.calibration_trust = PathAssessment::CALIBRATION_TRUST_ASSUMED;
  ObstacleTrack far, near, unconfirmed;
  far.track_id = 7;
  far.confirmed = true;
  far.distance_m = 40.0;
  near.track_id = 3;
  near.confirmed = true;
  near.distance_m = 18.9;
  near.hits = 4;
  near.bbox.size.x = 2.0;
  near.bbox.size.y = 2.0;
  near.bbox.size.z = 1.5;
  unconfirmed.track_id = 9;
  assessment.tracks = {far, near, unconfirmed};

  const auto text = assessment_text(assessment);
  EXPECT_NE(text.find("state: OBSTACLE\n"), std::string::npos);
  EXPECT_NE(text.find("distance_m: 18.9\n"), std::string::npos);
  EXPECT_NE(text.find("calibration_trust: ASSUMED\n"), std::string::npos);
  EXPECT_NE(text.find("- id 3: 18.9 m, 2.0 x 2.0 x 1.5 m, hits 4\n"), std::string::npos);
  EXPECT_LT(text.find("- id 3"), text.find("- id 7"));
  EXPECT_EQ(text.find("id 9"), std::string::npos);
}

TEST(AssessmentText, MissingDistanceIsNeverShownAsANumber) {
  PathAssessment assessment;
  assessment.distance_m = std::nan("");
  const auto text = assessment_text(assessment);
  EXPECT_NE(text.find("state: UNKNOWN\n"), std::string::npos);
  EXPECT_NE(text.find("distance_m: - (distance_valid: false)\n"), std::string::npos);
  EXPECT_NE(text.find("evaluated_range_m: - (evaluation_region_valid: false)\n"),
            std::string::npos);
  EXPECT_NE(text.find("confirmed_tracks: []\n"), std::string::npos);
}
