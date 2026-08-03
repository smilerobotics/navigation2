// Copyright (c) 2026 SmileRobotics, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "nav2_amcl/scan_deskew.hpp"

#include <cmath>
#include <vector>

namespace nav2_amcl
{

bool
deskewScanRanges(
  const std::vector<float> & input_ranges, float time_increment,
  const DeskewPose2D & base_at_start, const DeskewPose2D & base_at_end,
  double laser_x_in_base, double laser_y_in_base,
  double angle_min, double angle_increment,
  double range_min, double range_max,
  double(*ranges)[2])
{
  const int range_count = static_cast<int>(input_ranges.size());
  if (time_increment <= 0.0f || range_count < 2) {
    return false;
  }

  const double start_x = base_at_start.x;
  const double start_y = base_at_start.y;
  const double start_yaw = base_at_start.yaw;
  const double delta_x = base_at_end.x - start_x;
  const double delta_y = base_at_end.y - start_y;
  double delta_yaw = base_at_end.yaw - start_yaw;
  delta_yaw = std::atan2(std::sin(delta_yaw), std::cos(delta_yaw));

  const double cos_start = std::cos(start_yaw);
  const double sin_start = std::sin(start_yaw);
  // Laser origin in odom at the reference (scan stamp) pose.
  const double ref_origin_x =
    start_x + cos_start * laser_x_in_base - sin_start * laser_y_in_base;
  const double ref_origin_y =
    start_y + sin_start * laser_x_in_base + cos_start * laser_y_in_base;

  const double denom = static_cast<double>(range_count - 1);
  for (int i = 0; i < range_count; i++) {
    const double bearing_base = angle_min + (i * angle_increment);
    const double input_range = input_ranges[i];
    if (!std::isfinite(input_range) || input_range <= range_min || input_range >= range_max) {
      // Out-of-range beams must stay max-range markers: re-anchoring a
      // finite range_max reading can shrink it below the sensor model's
      // max-range cutoff and turn it into a phantom obstacle.
      ranges[i][0] = range_max;
      ranges[i][1] = bearing_base;
      continue;
    }
    const double fraction = i / denom;
    const double yaw_i = start_yaw + fraction * delta_yaw;
    const double origin_x_i = start_x + fraction * delta_x +
      std::cos(yaw_i) * laser_x_in_base - std::sin(yaw_i) * laser_y_in_base;
    const double origin_y_i = start_y + fraction * delta_y +
      std::sin(yaw_i) * laser_x_in_base + std::cos(yaw_i) * laser_y_in_base;
    const double beam_dir = yaw_i + bearing_base;
    const double endpoint_x = origin_x_i + input_range * std::cos(beam_dir);
    const double endpoint_y = origin_y_i + input_range * std::sin(beam_dir);
    // Vector from the reference laser origin to the endpoint, in the reference
    // (base-at-stamp) orientation.
    const double vector_x = endpoint_x - ref_origin_x;
    const double vector_y = endpoint_y - ref_origin_y;
    const double ref_x = cos_start * vector_x + sin_start * vector_y;
    const double ref_y = -sin_start * vector_x + cos_start * vector_y;
    ranges[i][0] = std::hypot(ref_x, ref_y);
    ranges[i][1] = std::atan2(ref_y, ref_x);
  }
  return true;
}

}  // namespace nav2_amcl
