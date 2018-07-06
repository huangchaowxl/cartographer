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

#include "cartographer/mapping_2d/local_trajectory_builder.h"

#include <limits>

#include "cartographer/common/make_unique.h"
#include "cartographer/sensor/laser.h"
#include "iostream"
#include <fstream>

using namespace std;




namespace cartographer {
namespace mapping_2d {

proto::LocalTrajectoryBuilderOptions CreateLocalTrajectoryBuilderOptions(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::LocalTrajectoryBuilderOptions options;
  options.set_horizontal_laser_min_z(
      parameter_dictionary->GetDouble("horizontal_laser_min_z"));
  options.set_horizontal_laser_max_z(
      parameter_dictionary->GetDouble("horizontal_laser_max_z"));
  options.set_horizontal_laser_voxel_filter_size(
      parameter_dictionary->GetDouble("horizontal_laser_voxel_filter_size"));
  options.set_use_online_correlative_scan_matching(
      parameter_dictionary->GetBool("use_online_correlative_scan_matching"));
  *options.mutable_adaptive_voxel_filter_options() =
      sensor::CreateAdaptiveVoxelFilterOptions(
          parameter_dictionary->GetDictionary("adaptive_voxel_filter").get());
  *options.mutable_real_time_correlative_scan_matcher_options() =
      scan_matching::CreateRealTimeCorrelativeScanMatcherOptions(
          parameter_dictionary
              ->GetDictionary("real_time_correlative_scan_matcher")
              .get());
  *options.mutable_ceres_scan_matcher_options() =
      scan_matching::CreateCeresScanMatcherOptions(
          parameter_dictionary->GetDictionary("ceres_scan_matcher").get());
  *options.mutable_motion_filter_options() =
      mapping_3d::CreateMotionFilterOptions(
          parameter_dictionary->GetDictionary("motion_filter").get());
  *options.mutable_pose_tracker_options() =
      kalman_filter::CreatePoseTrackerOptions(
          parameter_dictionary->GetDictionary("pose_tracker").get());
  *options.mutable_submaps_options() = CreateSubmapsOptions(
      parameter_dictionary->GetDictionary("submaps").get());
  options.set_use_imu_data(parameter_dictionary->GetBool("use_imu_data"));
  return options;
}

LocalTrajectoryBuilder::LocalTrajectoryBuilder(
    const proto::LocalTrajectoryBuilderOptions& options)
    : options_(options),
      submaps_(options.submaps_options()),
      scan_matcher_pose_estimate_(transform::Rigid3d::Identity()),
      motion_filter_(options_.motion_filter_options()),
      real_time_correlative_scan_matcher_(
          options_.real_time_correlative_scan_matcher_options()),
      ceres_scan_matcher_(options_.ceres_scan_matcher_options()) ,
     full_map_grid(cartographer::mapping_2d:: MapLimits(0.05, Eigen::Vector2d(200, 200),cartographer::mapping_2d:: CellLimits(8000, 8000))),
      read_grid(0){}

LocalTrajectoryBuilder::~LocalTrajectoryBuilder() {}

const Submaps* LocalTrajectoryBuilder::submaps() const { return &submaps_; }

Submaps* LocalTrajectoryBuilder::submaps() { return &submaps_; }

kalman_filter::PoseTracker* LocalTrajectoryBuilder::pose_tracker() const {
  return pose_tracker_.get();
}

sensor::LaserFan LocalTrajectoryBuilder::BuildProjectedLaserFan(
    const transform::Rigid3f& tracking_to_tracking_2d,
    const sensor::LaserFan3D& laser_fan) const {
  const sensor::LaserFan projected_fan = sensor::ProjectCroppedLaserFan(
      sensor::TransformLaserFan3D(laser_fan, tracking_to_tracking_2d),
      Eigen::Vector3f(-std::numeric_limits<float>::infinity(),
                      -std::numeric_limits<float>::infinity(),
                      options_.horizontal_laser_min_z()),
      Eigen::Vector3f(std::numeric_limits<float>::infinity(),
                      std::numeric_limits<float>::infinity(),
                      options_.horizontal_laser_max_z()));
  return sensor::LaserFan{
      projected_fan.origin,
      sensor::VoxelFiltered(projected_fan.point_cloud,
                            options_.horizontal_laser_voxel_filter_size()),
      sensor::VoxelFiltered(projected_fan.missing_echo_point_cloud,
                            options_.horizontal_laser_voxel_filter_size())};
}
int count_frame_2=0;
void LocalTrajectoryBuilder::ScanMatch(
    common::Time time, const transform::Rigid3d& pose_prediction,
    const transform::Rigid3d& tracking_to_tracking_2d,
    const sensor::LaserFan& laser_fan_in_tracking_2d,
    transform::Rigid3d* pose_observation,
    kalman_filter::PoseCovariance* covariance_observation,bool offline) {
  const ProbabilityGrid& probability_grid =
      submaps_.Get(submaps_.matching_index())->probability_grid;
//  const ProbabilityGrid& probability_grid =full_map_grid;
 count_frame_2=count_frame_2+1;
  if (count_frame_2==90)
  {
      cout<<"full_probability_grid .x_max :  "<<   probability_grid.get_max_x()<<endl;
      cout<<"full_probability_grid .y_max :  "<<   probability_grid.get_max_y()<<endl;
      cout<<"full_probability_grid .x_min :  "<<   probability_grid.get_min_x()<<endl;
      cout<<"full_probability_grid .y_min :  "<<   probability_grid.get_min_y()<<endl;
      count_frame_2=0;
  }
 transform::Rigid2d pose_prediction_2d =
      transform::Project2D(pose_prediction * tracking_to_tracking_2d.inverse());
  // The online correlative scan matcher will refine the initial estimate for
  // the Ceres scan matcher.
  transform::Rigid2d initial_ceres_pose = pose_prediction_2d;
  sensor::AdaptiveVoxelFilter adaptive_voxel_filter(
      options_.adaptive_voxel_filter_options());
  const sensor::PointCloud2D filtered_point_cloud_in_tracking_2d =
      adaptive_voxel_filter.Filter(laser_fan_in_tracking_2d.point_cloud);
  if (options_.use_online_correlative_scan_matching()) {

    double r_match_score;
    r_match_score=real_time_correlative_scan_matcher_.Match(
        pose_prediction_2d, filtered_point_cloud_in_tracking_2d,
        probability_grid, &initial_ceres_pose);
    if (count_frame_2==0)
     {
        cout<<"r_match_score   :   "<<r_match_score<<endl;
     }
  }

  transform::Rigid2d tracking_2d_to_map;
  kalman_filter::Pose2DCovariance covariance_observation_2d;
  ceres::Solver::Summary summary;
  ceres_scan_matcher_.Match(
      transform::Project2D(scan_matcher_pose_estimate_ *
                           tracking_to_tracking_2d.inverse()),
      initial_ceres_pose, filtered_point_cloud_in_tracking_2d, probability_grid,
      &tracking_2d_to_map, &covariance_observation_2d, &summary);

  CHECK(pose_tracker_ != nullptr);

  *pose_observation = transform::Embed3D(tracking_2d_to_map);
  // This covariance is used for non-yaw rotation and the fake height of 0.
  constexpr double kFakePositionCovariance = 1.;
  constexpr double kFakeOrientationCovariance = 1.;
  *covariance_observation =
      kalman_filter::Embed3D(covariance_observation_2d, kFakePositionCovariance,
                             kFakeOrientationCovariance);
  pose_tracker_->AddPoseObservation(
      time, (*pose_observation) * tracking_to_tracking_2d,
      *covariance_observation);
}

std::unique_ptr<LocalTrajectoryBuilder::InsertionResult>
LocalTrajectoryBuilder::AddHorizontalLaserFan(
    const common::Time time, const sensor::LaserFan3D& laser_fan) {

  if(read_grid)
  {
      read_grid_from_disk();
  }
  // Initialize pose tracker now if we do not ever use an IMU.
  if (!options_.use_imu_data()) {
    InitializePoseTracker(time);
  }

  if (pose_tracker_ == nullptr) {
    // Until we've initialized the UKF with our first IMU message, we cannot
    // compute the orientation of the laser scanner.
    LOG(INFO) << "PoseTracker not yet initialized.";
    return nullptr;
  }

  transform::Rigid3d pose_prediction;
  kalman_filter::PoseCovariance covariance_prediction;
  pose_tracker_->GetPoseEstimateMeanAndCovariance(time, &pose_prediction,
                                                  &covariance_prediction);

  // Computes the rotation without yaw, as defined by GetYaw().
  const transform::Rigid3d tracking_to_tracking_2d =
      transform::Rigid3d::Rotation(
          Eigen::Quaterniond(Eigen::AngleAxisd(
              -transform::GetYaw(pose_prediction), Eigen::Vector3d::UnitZ())) *
          pose_prediction.rotation());
/*  cout<<"tracking_to_tracking_2d.translation : "<<tracking_to_tracking_2d.translation()(0,0)<<" "
  <<tracking_to_tracking_2d.translation()(1,0)<<" "
  <<tracking_to_tracking_2d.translation()(2,0)<<" "<<endl;
  cout<<"tracking_to_tracking_2d.translation : "<<tracking_to_tracking_2d.rotation().w()<<" "
  <<tracking_to_tracking_2d.rotation().x()<<" "
  <<tracking_to_tracking_2d.rotation().y()<<" "
  <<tracking_to_tracking_2d.rotation().z()<<" "<<endl;*/
  const sensor::LaserFan laser_fan_in_tracking_2d =
      BuildProjectedLaserFan(tracking_to_tracking_2d.cast<float>(), laser_fan);

  if (laser_fan_in_tracking_2d.point_cloud.empty()) {
    LOG(WARNING) << "Dropped empty horizontal laser point cloud.";
    return nullptr;
  }

  transform::Rigid3d pose_observation;
  kalman_filter::PoseCovariance covariance_observation;
  ScanMatch(time, pose_prediction, tracking_to_tracking_2d,
            laser_fan_in_tracking_2d, &pose_observation,
            &covariance_observation,0);

  kalman_filter::PoseCovariance covariance_estimate;
  pose_tracker_->GetPoseEstimateMeanAndCovariance(
      time, &scan_matcher_pose_estimate_, &covariance_estimate);

  // Remove the untracked z-component which floats around 0 in the UKF.
  const auto translation = scan_matcher_pose_estimate_.translation();
  scan_matcher_pose_estimate_ = transform::Rigid3d(
      transform::Rigid3d::Vector(translation.x(), translation.y(), 0.),
      scan_matcher_pose_estimate_.rotation());

  const transform::Rigid3d tracking_2d_to_map =
      scan_matcher_pose_estimate_ * tracking_to_tracking_2d.inverse();
  last_pose_estimate_ = {
      time,
      {pose_prediction, covariance_prediction},
      {pose_observation, covariance_observation},
      {scan_matcher_pose_estimate_, covariance_estimate},
      scan_matcher_pose_estimate_,
      sensor::TransformPointCloud(
          sensor::ToPointCloud(laser_fan_in_tracking_2d.point_cloud),
          tracking_2d_to_map.cast<float>())};

  const transform::Rigid2d pose_estimate_2d =
      transform::Project2D(tracking_2d_to_map);
  if (motion_filter_.IsSimilar(time, transform::Embed3D(pose_estimate_2d))) {
    return nullptr;
  }
  const mapping::Submap* const matching_submap =
      submaps_.Get(submaps_.matching_index());
  std::vector<const mapping::Submap*> insertion_submaps;
  for (int insertion_index : submaps_.insertion_indices()) {
    insertion_submaps.push_back(submaps_.Get(insertion_index));
  }

  submaps_.InsertLaserFan(TransformLaserFan(laser_fan_in_tracking_2d,
                                            pose_estimate_2d.cast<float>()));

  return common::make_unique<InsertionResult>(InsertionResult{
      time, &submaps_, matching_submap, insertion_submaps,
      tracking_to_tracking_2d, tracking_2d_to_map, laser_fan_in_tracking_2d,
      pose_estimate_2d, covariance_estimate});
}



const mapping::GlobalTrajectoryBuilderInterface::PoseEstimate&
LocalTrajectoryBuilder::pose_estimate() const {
  return last_pose_estimate_;
}

void LocalTrajectoryBuilder::AddImuData(
    const common::Time time, const Eigen::Vector3d& linear_acceleration,
    const Eigen::Vector3d& angular_velocity) {
  CHECK(options_.use_imu_data()) << "An unexpected IMU packet was added.";

  InitializePoseTracker(time);
  pose_tracker_->AddImuLinearAccelerationObservation(time, linear_acceleration);
  pose_tracker_->AddImuAngularVelocityObservation(time, angular_velocity);

  transform::Rigid3d pose_estimate;
  kalman_filter::PoseCovariance unused_covariance_estimate;
  pose_tracker_->GetPoseEstimateMeanAndCovariance(time, &pose_estimate,
                                                  &unused_covariance_estimate);

  // Log a warning if the backpack inclination exceeds 20 degrees. In these
  // cases, it's very likely that 2D SLAM will fail.
  const Eigen::Vector3d gravity_direction =
      Eigen::Quaterniond(pose_estimate.rotation()) * Eigen::Vector3d::UnitZ();
  const double inclination = std::acos(gravity_direction.z());
  constexpr double kMaxInclination = common::DegToRad(20.);
  LOG_IF_EVERY_N(WARNING, inclination > kMaxInclination, 1000)
      << "Max inclination exceeded: " << common::RadToDeg(inclination) << " > "
      << common::RadToDeg(kMaxInclination);
}

void LocalTrajectoryBuilder::AddOdometerPose(
    const common::Time time, const transform::Rigid3d& pose,
    const kalman_filter::PoseCovariance& covariance) {
  if (pose_tracker_ == nullptr) {
    // Until we've initialized the UKF with our first IMU message, we cannot
    // process odometry poses.
    LOG_EVERY_N(INFO, 100) << "PoseTracker not yet initialized.";
  } else {
    pose_tracker_->AddOdometerPoseObservation(time, pose, covariance);
    cout<<"test  odometer data"<<endl;
  }
}

void LocalTrajectoryBuilder::InitializePoseTracker(const common::Time time) {
  if (pose_tracker_ == nullptr) {
    pose_tracker_ = common::make_unique<kalman_filter::PoseTracker>(
        options_.pose_tracker_options(),
        kalman_filter::PoseTracker::ModelFunction::k2D, time);
  }
}
void LocalTrajectoryBuilder::read_grid_from_disk()
{
    if(read_grid)
    {
        read_grid=0;
        ifstream infile;
        ifstream infile_2;
        int num_x=0,num_y=0,offset_x=0,offset_y=0;
        infile.open("/home/wxl/ros_slam/catkin_ws/src/cartographer_ros/my_package/src/data/2018_06_01 14_51_17.txt");
        infile_2.open("/home/wxl/ros_slam/catkin_ws/src/cartographer_ros/my_package/src/data/2018_06_01 14_51_17_2.txt");
        string s1,s2,s3,s4;
        getline(infile_2,s1);
        getline(infile_2,s2);
        getline(infile_2,s3);
        getline(infile_2,s4);
        num_x=std::stoi (s1,nullptr,10);
        num_y=std::stoi (s2,nullptr,10);
        offset_x=std::stoi (s3,nullptr,10);
        offset_y=std::stoi (s4,nullptr,10);
        full_map_grid.StartUpdate();
        cartographer::mapping_2d::CellLimits cell_limits_2(num_x-offset_x+1, num_y-offset_y+1);
        Eigen::Array2i offset_2(3829,3567);
        cout<<"probability_grid_size  :  "<<num_x<<"  "<<num_y<<"  "<<offset_x<<"  "<<offset_y<<endl;
        for (const Eigen::Array2i& xy_index :
           cartographer::mapping_2d::XYIndexRangeIterator(cell_limits_2)) {
            string s;
            getline(infile,s);
            int data=std::stoi (s,nullptr,10);
            float data_2=data;
            float p=0.0;
            if (data!=-1)
            {
                p= (cartographer::mapping::kMaxProbability - cartographer::mapping::kMinProbability)* data_2/100.0+cartographer::mapping::kMinProbability;
                full_map_grid.SetProbability(xy_index+offset_2,p);
            }
        }
        infile.close();
        infile_2.close();
        cout<<"finish full_map_grid----------------->"<<endl;
        }
}


void LocalTrajectoryBuilder::locate_record_pose(const transform::Rigid2d pose_estimate_2d)
{


}
}  // namespace mapping_2d
}  // namespace cartographer
