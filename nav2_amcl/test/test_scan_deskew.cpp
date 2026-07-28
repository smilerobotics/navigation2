/*
 *  Copyright (c) 2026, SmileRobotics, Inc.
 *  All rights reserved.
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <cmath>
#include <limits>
#include <vector>

#include "gtest/gtest.h"
#include "nav2_amcl/scan_deskew.hpp"

namespace
{

constexpr double kTolerance = 1e-9;
constexpr float kTimeIncrement = 0.001f;
constexpr double kRangeMin = 0.1;
constexpr double kRangeMax = 10.0;

// Physical reference model, written independently of the implementation:
// the beam is emitted from the laser origin at the interpolated base pose
// and its endpoint is re-expressed about the laser origin at the start pose.
void expectedRangeBearing(
  int beam_index, int beam_count,
  const nav2_amcl::DeskewPose2D & start, const nav2_amcl::DeskewPose2D & end,
  double laser_x, double laser_y, double bearing, double range,
  double & expected_range, double & expected_bearing)
{
  const double fraction =
    static_cast<double>(beam_index) / static_cast<double>(beam_count - 1);
  double delta_yaw = end.yaw - start.yaw;
  delta_yaw = std::atan2(std::sin(delta_yaw), std::cos(delta_yaw));
  const double yaw_i = start.yaw + fraction * delta_yaw;
  const double base_x_i = start.x + fraction * (end.x - start.x);
  const double base_y_i = start.y + fraction * (end.y - start.y);
  const double origin_x_i = base_x_i + std::cos(yaw_i) * laser_x - std::sin(yaw_i) * laser_y;
  const double origin_y_i = base_y_i + std::sin(yaw_i) * laser_x + std::cos(yaw_i) * laser_y;
  const double endpoint_x = origin_x_i + range * std::cos(yaw_i + bearing);
  const double endpoint_y = origin_y_i + range * std::sin(yaw_i + bearing);
  const double ref_origin_x =
    start.x + std::cos(start.yaw) * laser_x - std::sin(start.yaw) * laser_y;
  const double ref_origin_y =
    start.y + std::sin(start.yaw) * laser_x + std::cos(start.yaw) * laser_y;
  const double dx = endpoint_x - ref_origin_x;
  const double dy = endpoint_y - ref_origin_y;
  const double ref_x = std::cos(start.yaw) * dx + std::sin(start.yaw) * dy;
  const double ref_y = -std::sin(start.yaw) * dx + std::cos(start.yaw) * dy;
  expected_range = std::hypot(ref_x, ref_y);
  expected_bearing = std::atan2(ref_y, ref_x);
}

}  // namespace

TEST(ScanDeskew, NoMotionMatchesRigidProjection)
{
  const nav2_amcl::DeskewPose2D pose{1.0, -2.0, 0.7};
  const std::vector<float> input = {2.0f, 3.0f, 4.0f};
  double out[3][2];
  ASSERT_TRUE(
    nav2_amcl::deskewScanRanges(
      input, kTimeIncrement, pose, pose, 0.2, -0.1, -0.3, 0.15, kRangeMin, kRangeMax, out));
  for (int i = 0; i < 3; i++) {
    EXPECT_NEAR(out[i][0], input[i], kTolerance) << "beam " << i;
    EXPECT_NEAR(out[i][1], -0.3 + i * 0.15, kTolerance) << "beam " << i;
  }
}

TEST(ScanDeskew, PureTranslationReanchorsEndpoints)
{
  const nav2_amcl::DeskewPose2D start{0.0, 0.0, 0.0};
  const nav2_amcl::DeskewPose2D end{0.3, 0.0, 0.0};
  const std::vector<float> input = {2.0f, 2.0f, 2.0f};
  double out[3][2];
  // Beams along +x: the interpolated origin shift adds directly to the range.
  ASSERT_TRUE(
    nav2_amcl::deskewScanRanges(
      input, kTimeIncrement, start, end, 0.0, 0.0, 0.0, 0.0, kRangeMin, kRangeMax, out));
  EXPECT_NEAR(out[0][0], 2.0, kTolerance);
  EXPECT_NEAR(out[1][0], 2.15, kTolerance);
  EXPECT_NEAR(out[2][0], 2.3, kTolerance);
  EXPECT_NEAR(out[2][1], 0.0, kTolerance);

  // Beams along +y: the shift is lateral, so both range and bearing change.
  ASSERT_TRUE(
    nav2_amcl::deskewScanRanges(
      input, kTimeIncrement, start, end, 0.0, 0.0, M_PI / 2.0, 0.0, kRangeMin, kRangeMax, out));
  EXPECT_NEAR(out[2][0], std::hypot(0.3, 2.0), kTolerance);
  EXPECT_NEAR(out[2][1], std::atan2(2.0, 0.3), kTolerance);
}

TEST(ScanDeskew, PureRotationShiftsBearing)
{
  const nav2_amcl::DeskewPose2D start{0.0, 0.0, 0.0};
  const nav2_amcl::DeskewPose2D end{0.0, 0.0, 0.2};
  const std::vector<float> input = {2.0f, 2.0f, 2.0f};
  double out[3][2];
  ASSERT_TRUE(
    nav2_amcl::deskewScanRanges(
      input, kTimeIncrement, start, end, 0.0, 0.0, 0.1, 0.0, kRangeMin, kRangeMax, out));
  // With the laser at the rotation center the range is preserved and the
  // bearing picks up the interpolated yaw.
  EXPECT_NEAR(out[0][0], 2.0, kTolerance);
  EXPECT_NEAR(out[0][1], 0.1, kTolerance);
  EXPECT_NEAR(out[1][1], 0.2, kTolerance);
  EXPECT_NEAR(out[2][0], 2.0, kTolerance);
  EXPECT_NEAR(out[2][1], 0.3, kTolerance);
}

TEST(ScanDeskew, LaserMountOffsetIsAccountedFor)
{
  const nav2_amcl::DeskewPose2D start{0.4, -0.3, 0.5};
  const nav2_amcl::DeskewPose2D end{0.45, -0.32, 0.7};
  const double laser_x = 0.2;
  const double laser_y = 0.1;
  const double angle_min = -0.3;
  const double angle_increment = 0.15;
  const std::vector<float> input = {1.5f, 1.5f, 1.5f, 1.5f, 1.5f};
  double out[5][2];
  ASSERT_TRUE(
    nav2_amcl::deskewScanRanges(
      input, kTimeIncrement, start, end, laser_x, laser_y, angle_min, angle_increment,
      kRangeMin, kRangeMax, out));
  for (int i = 0; i < 5; i++) {
    double expected_range = 0.0;
    double expected_bearing = 0.0;
    expectedRangeBearing(
      i, 5, start, end, laser_x, laser_y, angle_min + i * angle_increment, input[i],
      expected_range, expected_bearing);
    EXPECT_NEAR(out[i][0], expected_range, kTolerance) << "beam " << i;
    EXPECT_NEAR(out[i][1], expected_bearing, kTolerance) << "beam " << i;
  }
}

TEST(ScanDeskew, YawWrapInterpolatesAlongShortestArc)
{
  // Physically a +0.1 rad turn across the +/-pi seam. A naive (unwrapped)
  // delta of -2*pi + 0.1 would swing the mid-sweep pose the other way around.
  const nav2_amcl::DeskewPose2D start{0.0, 0.0, M_PI - 0.05};
  const nav2_amcl::DeskewPose2D end{0.0, 0.0, -M_PI + 0.05};
  const std::vector<float> input = {2.0f, 2.0f, 2.0f};
  double out[3][2];
  ASSERT_TRUE(
    nav2_amcl::deskewScanRanges(
      input, kTimeIncrement, start, end, 0.0, 0.0, 0.0, 0.0, kRangeMin, kRangeMax, out));
  EXPECT_NEAR(out[1][0], 2.0, kTolerance);
  EXPECT_NEAR(out[1][1], 0.05, kTolerance);
  EXPECT_NEAR(out[2][1], 0.1, kTolerance);
}

TEST(ScanDeskew, OutOfRangeBeamsStayMaxRangeMarkers)
{
  const nav2_amcl::DeskewPose2D start{0.0, 0.0, 0.0};
  const nav2_amcl::DeskewPose2D end{0.3, 0.0, 0.2};
  const double angle_min = -0.2;
  const double angle_increment = 0.1;
  const std::vector<float> input = {
    std::numeric_limits<float>::quiet_NaN(),
    std::numeric_limits<float>::infinity(),
    0.05f,   // below range_min
    10.0f,   // at range_max
    11.0f};  // above range_max
  double out[5][2];
  ASSERT_TRUE(
    nav2_amcl::deskewScanRanges(
      input, kTimeIncrement, start, end, 0.0, 0.0, angle_min, angle_increment,
      kRangeMin, kRangeMax, out));
  for (int i = 0; i < 5; i++) {
    EXPECT_EQ(out[i][0], kRangeMax) << "beam " << i;
    EXPECT_NEAR(out[i][1], angle_min + i * angle_increment, kTolerance) << "beam " << i;
  }
}

TEST(ScanDeskew, RejectsDegenerateScans)
{
  const nav2_amcl::DeskewPose2D start{0.0, 0.0, 0.0};
  const nav2_amcl::DeskewPose2D end{0.3, 0.0, 0.0};
  double out[3][2];
  const std::vector<float> single_beam = {2.0f};
  EXPECT_FALSE(
    nav2_amcl::deskewScanRanges(
      single_beam, kTimeIncrement, start, end, 0.0, 0.0, 0.0, 0.0, kRangeMin, kRangeMax, out));
  const std::vector<float> input = {2.0f, 2.0f, 2.0f};
  EXPECT_FALSE(
    nav2_amcl::deskewScanRanges(
      input, 0.0f, start, end, 0.0, 0.0, 0.0, 0.0, kRangeMin, kRangeMax, out));
  EXPECT_FALSE(
    nav2_amcl::deskewScanRanges(
      input, -0.001f, start, end, 0.0, 0.0, 0.0, 0.0, kRangeMin, kRangeMax, out));
}
