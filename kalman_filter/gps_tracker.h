/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_KALMAN_FILTER_GPS_TRACKER_H_
#define CARTOGRAPHER_KALMAN_FILTER_GPS_TRACKER_H_

#include <deque>

#include "cartographer/common/time.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace kalman_filter {

struct Gps_tracker {
  Gps_tracker(common::Time time, const transform::Rigid2d& gps_pose);
  Gps_tracker() {}

  int64 time =0;
  transform::Rigid2d gps_pose = transform::Rigid2d::Identity();
  transform::Rigid2d gps_covariance =transform::Rigid2d::Identity();
  int32 gps_flag=0;
};

// Keeps track of the odometry states by keeping sliding window over some
// number of them.
}  // namespace kalman_filter
}  // namespace cartographer

#endif  // CARTOGRAPHER_KALMAN_FILTER_ODOMETRY_STATE_TRACKER_H_
