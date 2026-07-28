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

#ifndef NAV2_AMCL__SCAN_DESKEW_HPP_
#define NAV2_AMCL__SCAN_DESKEW_HPP_

#include <vector>

namespace nav2_amcl
{

/// 2D base pose in the odom frame used for the scan deskew interpolation.
struct DeskewPose2D
{
  double x;
  double y;
  double yaw;
};

/*
 * @brief Re-express the beams of a moving-sensor scan about a single origin.
 *
 * The measurement update treats every beam as measured from the base pose at
 * the scan stamp. This helper interpolates the base motion between the
 * first-beam and last-beam poses, computes each beam's true endpoint, and
 * re-expresses it as (range, bearing) about the laser origin at the scan
 * stamp, so the single-origin model reconstructs the true endpoints despite
 * the base having moved during the sweep.
 *
 * Beams that are non-finite, at or below range_min, or at or above range_max
 * are kept as max-range markers with their rigid bearing: re-anchoring a
 * finite range_max reading could shrink it below the sensor model's
 * max-range cutoff and turn it into a phantom obstacle.
 *
 * @param input_ranges Raw scan ranges (beam order, first beam at the stamp)
 * @param time_increment Seconds between consecutive beams
 * @param base_at_start Base pose in odom at the first-beam time
 * @param base_at_end Base pose in odom at the last-beam time
 * @param laser_x_in_base Laser origin x in the base frame
 * @param laser_y_in_base Laser origin y in the base frame
 * @param angle_min First-beam bearing in the base frame
 * @param angle_increment Bearing step between consecutive beams
 * @param range_min Minimum valid range
 * @param range_max Maximum valid range (marker value for invalid beams)
 * @param ranges Output (range, bearing) pairs, sized to input_ranges.size()
 * @return false when the scan cannot be deskewed (fewer than two beams or a
 * non-positive time increment); the caller should keep the rigid projection.
 */
bool deskewScanRanges(
  const std::vector<float> & input_ranges, float time_increment,
  const DeskewPose2D & base_at_start, const DeskewPose2D & base_at_end,
  double laser_x_in_base, double laser_y_in_base,
  double angle_min, double angle_increment,
  double range_min, double range_max,
  double(*ranges)[2]);

}  // namespace nav2_amcl

#endif  // NAV2_AMCL__SCAN_DESKEW_HPP_
