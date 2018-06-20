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

#ifndef CARTOGRAPHER_MAPPING_2D_LOCATION_H_
#define CARTOGRAPHER_MAPPING_2D_LOCATION_H_

#include <memory>
#include <vector>

#include "Eigen/Core"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/mapping/proto/submaps.pb.h"
#include "cartographer/mapping/submaps.h"
#include "cartographer/mapping/trajectory_node.h"
#include "cartographer/mapping_2d/laser_fan_inserter.h"
#include "cartographer/mapping_2d/map_limits.h"
#include "cartographer/mapping_2d/probability_grid.h"
#include "cartographer/mapping_2d/proto/submaps_options.pb.h"
#include "cartographer/sensor/laser.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/kalman_filter/pose_tracker.h"
namespace cartographer {
namespace mapping_2d {

// A container of Submaps.
class location {
 public:

  location ();
  void pure_location();

  std::vector<double> get_location();
 private:

  std::vector<double> my_location;
  std::unique_ptr<kalman_filter::PoseTracker> pose_tracker_;
};

}  // namespace mapping_2d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_SUBMAPS_H_
