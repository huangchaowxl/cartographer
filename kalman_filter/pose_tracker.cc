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

#include "cartographer/kalman_filter/pose_tracker.h"

#include <cmath>
#include <limits>
#include <utility>

#include "Eigen/Geometry"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/math.h"
#include "cartographer/common/time.h"
#include "cartographer/kalman_filter/gaussian_distribution.h"
#include "cartographer/kalman_filter/unscented_kalman_filter.h"
#include "cartographer/kalman_filter/gps_tracker.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"
#include <iostream>
using namespace std;
namespace cartographer {
namespace kalman_filter {

namespace {

PoseTracker::State AddDelta(const PoseTracker::State& state,
                            const PoseTracker::State& delta) {
  PoseTracker::State new_state = state + delta;
  const Eigen::Quaterniond orientation =
      transform::AngleAxisVectorToRotationQuaternion(
          Eigen::Vector3d(state[PoseTracker::kMapOrientationX],
                          state[PoseTracker::kMapOrientationY],
                          state[PoseTracker::kMapOrientationZ]));
  const Eigen::Vector3d rotation_vector(delta[PoseTracker::kMapOrientationX],
                                        delta[PoseTracker::kMapOrientationY],
                                        delta[PoseTracker::kMapOrientationZ]);
  CHECK_LT(rotation_vector.norm(), M_PI / 2.)
      << "Sigma point is far from the mean, recovered delta may be incorrect.";
  const Eigen::Quaterniond rotation =
      transform::AngleAxisVectorToRotationQuaternion(rotation_vector);
  const Eigen::Vector3d new_orientation =
      transform::RotationQuaternionToAngleAxisVector(orientation * rotation);
  new_state[PoseTracker::kMapOrientationX] = new_orientation.x();
  new_state[PoseTracker::kMapOrientationY] = new_orientation.y();
  new_state[PoseTracker::kMapOrientationZ] = new_orientation.z();
  return new_state;
}

PoseTracker::State ComputeDelta(const PoseTracker::State& origin,
                                const PoseTracker::State& target) {
  PoseTracker::State delta = target - origin;
  const Eigen::Quaterniond origin_orientation =
      transform::AngleAxisVectorToRotationQuaternion(
          Eigen::Vector3d(origin[PoseTracker::kMapOrientationX],
                          origin[PoseTracker::kMapOrientationY],
                          origin[PoseTracker::kMapOrientationZ]));
  const Eigen::Quaterniond target_orientation =
      transform::AngleAxisVectorToRotationQuaternion(
          Eigen::Vector3d(target[PoseTracker::kMapOrientationX],
                          target[PoseTracker::kMapOrientationY],
                          target[PoseTracker::kMapOrientationZ]));
  const Eigen::Vector3d rotation =
      transform::RotationQuaternionToAngleAxisVector(
          origin_orientation.inverse() * target_orientation);
  delta[PoseTracker::kMapOrientationX] = rotation.x();
  delta[PoseTracker::kMapOrientationY] = rotation.y();
  delta[PoseTracker::kMapOrientationZ] = rotation.z();
  return delta;
}

// Build a model matrix for the given time delta.
PoseTracker::State ModelFunction3D(const PoseTracker::State& state,
                                   const double delta_t) {
  CHECK_GT(delta_t, 0.);

  PoseTracker::State new_state;
  new_state[PoseTracker::kMapPositionX] =
      state[PoseTracker::kMapPositionX] +
      delta_t * state[PoseTracker::kMapVelocityX];
  new_state[PoseTracker::kMapPositionY] =
      state[PoseTracker::kMapPositionY] +
      delta_t * state[PoseTracker::kMapVelocityY];
  new_state[PoseTracker::kMapPositionZ] =
      state[PoseTracker::kMapPositionZ] +
      delta_t * state[PoseTracker::kMapVelocityZ];

  new_state[PoseTracker::kMapOrientationX] =
      state[PoseTracker::kMapOrientationX];
  new_state[PoseTracker::kMapOrientationY] =
      state[PoseTracker::kMapOrientationY];
  new_state[PoseTracker::kMapOrientationZ] =
      state[PoseTracker::kMapOrientationZ];

  new_state[PoseTracker::kMapVelocityX] = state[PoseTracker::kMapVelocityX];
  new_state[PoseTracker::kMapVelocityY] = state[PoseTracker::kMapVelocityY];
  new_state[PoseTracker::kMapVelocityZ] = state[PoseTracker::kMapVelocityZ];

  return new_state;
}

// A specialization of ModelFunction3D that limits the z-component of position
// and velocity to 0.
PoseTracker::State ModelFunction2D(const PoseTracker::State& state,
                                   const double delta_t) {
  auto new_state = ModelFunction3D(state, delta_t);
  new_state[PoseTracker::kMapPositionZ] = 0.;
  new_state[PoseTracker::kMapVelocityZ] = 0.;
  new_state[PoseTracker::kMapOrientationX] = 0.;
  new_state[PoseTracker::kMapOrientationY] = 0.;
  return new_state;
}

}  // namespace

ImuTracker::ImuTracker(const proto::PoseTrackerOptions& options,
                       const common::Time time)
    : options_(options),
      time_(time),
      last_linear_acceleration_time_(common::Time::min()),
      orientation_(Eigen::Quaterniond::Identity()),
      gravity_direction_(Eigen::Vector3d::UnitZ()),
      imu_angular_velocity_(Eigen::Vector3d::Zero()) {}

void ImuTracker::Predict(const common::Time time) {
  CHECK_LE(time_, time);
  const double delta_t = common::ToSeconds(time - time_);
  const Eigen::Quaterniond rotation =
      transform::AngleAxisVectorToRotationQuaternion(
          Eigen::Vector3d(imu_angular_velocity_ * delta_t));
  orientation_ = (orientation_ * rotation).normalized();
  gravity_direction_ = rotation.inverse() * gravity_direction_;
  time_ = time;
}

void ImuTracker::AddImuLinearAccelerationObservation(
    const common::Time time, const Eigen::Vector3d& imu_linear_acceleration) {
  Predict(time);
  // Update the 'gravity_direction_' with an exponential moving average using
  // the 'imu_gravity_time_constant'.
  const double delta_t =
      last_linear_acceleration_time_ > common::Time::min()
          ? common::ToSeconds(time - last_linear_acceleration_time_)
          : std::numeric_limits<double>::infinity();
  last_linear_acceleration_time_ = time;
  const double alpha =
      1. - std::exp(-delta_t / options_.imu_gravity_time_constant());
  gravity_direction_ =
      (1. - alpha) * gravity_direction_ + alpha * imu_linear_acceleration;
  // Change the 'orientation_' so that it agrees with the current
  // 'gravity_direction_'.
  const Eigen::Quaterniond rotation = Eigen::Quaterniond::FromTwoVectors(
      gravity_direction_, orientation_.inverse() * Eigen::Vector3d::UnitZ());
  orientation_ = (orientation_ * rotation).normalized();
  CHECK_GT((orientation_.inverse() * Eigen::Vector3d::UnitZ())
               .dot(gravity_direction_),
           0.);
}

void ImuTracker::AddImuAngularVelocityObservation(
    const common::Time time, const Eigen::Vector3d& imu_angular_velocity) {
  Predict(time);
  imu_angular_velocity_ = imu_angular_velocity;
}

PoseAndCovariance operator*(const transform::Rigid3d& transform,
                            const PoseAndCovariance& pose_and_covariance) {
  GaussianDistribution<double, 6> distribution(
      Eigen::Matrix<double, 6, 1>::Zero(), pose_and_covariance.covariance);
  Eigen::Matrix<double, 6, 6> linear_transform;
  linear_transform << transform.rotation().matrix(), Eigen::Matrix3d::Zero(),
      Eigen::Matrix3d::Zero(), transform.rotation().matrix();
  return {transform * pose_and_covariance.pose,
          (linear_transform * distribution).GetCovariance()};
}

proto::PoseTrackerOptions CreatePoseTrackerOptions(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::PoseTrackerOptions options;
  options.set_position_model_variance(
      parameter_dictionary->GetDouble("position_model_variance"));
  options.set_orientation_model_variance(
      parameter_dictionary->GetDouble("orientation_model_variance"));
  options.set_velocity_model_variance(
      parameter_dictionary->GetDouble("velocity_model_variance"));
  options.set_imu_gravity_time_constant(
      parameter_dictionary->GetDouble("imu_gravity_time_constant"));
  options.set_imu_gravity_variance(
      parameter_dictionary->GetDouble("imu_gravity_variance"));
  options.set_num_odometry_states(
      parameter_dictionary->GetNonNegativeInt("num_odometry_states"));
  CHECK_GT(options.num_odometry_states(), 0);
  return options;
}

PoseTracker::Distribution PoseTracker::KalmanFilterInit() {
  State initial_state = State::Zero();
  // We are certain about the complete state at the beginning. We define the
  // initial pose to be at the origin and axis aligned. Additionally, we claim
  // that we are not moving.
  StateCovariance initial_covariance = 1e-9 * StateCovariance::Identity();
  return Distribution(initial_state, initial_covariance);
}

PoseTracker::PoseTracker(const proto::PoseTrackerOptions& options,
                         const ModelFunction& model_function,
                         const common::Time time)
    : options_(options),
      model_function_(model_function),
      time_(time),
      kalman_filter_(KalmanFilterInit(), AddDelta, ComputeDelta),
      imu_tracker_(options, time),
      odometry_state_tracker_(options.num_odometry_states()) {}

PoseTracker::~PoseTracker() {}

PoseTracker::Distribution PoseTracker::GetBelief(const common::Time time) {
  Predict(time);
  return kalman_filter_.GetBelief();
}

void PoseTracker::GetPoseEstimateMeanAndCovariance(const common::Time time,
                                                   transform::Rigid3d* pose,
                                                   PoseCovariance* covariance) {
  const Distribution belief = GetBelief(time);
  *pose = RigidFromState(belief.GetMean());
  static_assert(kMapPositionX == 0, "Cannot extract PoseCovariance.");
  static_assert(kMapPositionY == 1, "Cannot extract PoseCovariance.");
  static_assert(kMapPositionZ == 2, "Cannot extract PoseCovariance.");
  static_assert(kMapOrientationX == 3, "Cannot extract PoseCovariance.");
  static_assert(kMapOrientationY == 4, "Cannot extract PoseCovariance.");
  static_assert(kMapOrientationZ == 5, "Cannot extract PoseCovariance.");
  *covariance = belief.GetCovariance().block<6, 6>(0, 0);
  covariance->block<2, 2>(3, 3) +=
      options_.imu_gravity_variance() * Eigen::Matrix2d::Identity();
}

const PoseTracker::Distribution PoseTracker::BuildModelNoise(
    const double delta_t) const {
  // Position is constant, but orientation changes.
  StateCovariance model_noise = StateCovariance::Zero();

  model_noise.diagonal() <<
      // Position in map.
      options_.position_model_variance() * delta_t,
      options_.position_model_variance() * delta_t,
      options_.position_model_variance() * delta_t,

      // Orientation in map.
      options_.orientation_model_variance() * delta_t,
      options_.orientation_model_variance() * delta_t,
      options_.orientation_model_variance() * delta_t,

      // Linear velocities in map.
      options_.velocity_model_variance() * delta_t,
      options_.velocity_model_variance() * delta_t,
      options_.velocity_model_variance() * delta_t;

  return Distribution(State::Zero(), model_noise);
}

void PoseTracker::Predict(const common::Time time) {
  imu_tracker_.Predict(time);
  CHECK_LE(time_, time);
  const double delta_t = common::ToSeconds(time - time_);
  if (delta_t == 0.) {
    return;
  }
  kalman_filter_.Predict(
      [this, delta_t](const State& state) -> State {
        switch (model_function_) {
          case ModelFunction::k2D:
            return ModelFunction2D(state, delta_t);
          case ModelFunction::k3D:
            return ModelFunction3D(state, delta_t);
          default:
            LOG(FATAL);
        }
      },
      BuildModelNoise(delta_t));
  time_ = time;
}

void PoseTracker::AddImuLinearAccelerationObservation(
    const common::Time time, const Eigen::Vector3d& imu_linear_acceleration) {
  imu_tracker_.AddImuLinearAccelerationObservation(time,
                                                   imu_linear_acceleration);
  Predict(time);
}

void PoseTracker::AddImuAngularVelocityObservation(
    const common::Time time, const Eigen::Vector3d& imu_angular_velocity) {
  imu_tracker_.AddImuAngularVelocityObservation(time, imu_angular_velocity);
  Predict(time);
}
int count_pose=0;
int64 laser_tiemstamp_first=0,gps_tiemstamp_first=0;
void PoseTracker::AddPoseObservation(const common::Time time,
                                     const transform::Rigid3d& pose,
                                     const PoseCovariance& covariance) {
  Predict(time);
  Eigen::Matrix<double, 6, 1> gps_observation;

  if(laser_tiemstamp_first==0)
    {
        laser_tiemstamp_first=time.time_since_epoch().count();
    }
   if (gps_tiemstamp_first==0&&(gps_tracker.gps_flag==3||gps_tracker.gps_flag==4||gps_tracker.gps_flag==2))
   {
       gps_tiemstamp_first=gps_tracker.time;
   }
   int time_stamp_1=time.time_since_epoch().count()-laser_tiemstamp_first;
   time_stamp_1=time_stamp_1/10000;
   int time_stamp_2=gps_tracker.time-gps_tiemstamp_first;
  // Noise covariance is taken directly from the input values.
  //time_point <system_clock,duration<int>> tp_seconds (duration<int>(1));
 // cout<<"gps data stadus------>>>>>"<<time.time_since_epoch().count()<<"   "<<gps_tracker.gps_flag<<endl;

 // cout<<"time_stamp_1  :  "<<time_stamp_1<<" "<<laser_tiemstamp_first<<endl;
//  cout<<"time_stamp_2  :  "<<time_stamp_2<<" "<<gps_tiemstamp_first<<endl;
  const int delta_t = time_stamp_1 - time_stamp_2;

 // cout<<"gps data status : "<<delta_t<<"  "<<gps_tracker.gps_flag<<endl;


  //if (abs(delta_t) < 300&&(gps_tracker.gps_flag==3||gps_tracker.gps_flag==4))
   if(0)
    {
    cout<<"gps_data is ok"<<endl;
    gps_observation(0,0)=gps_tracker.gps_pose.translation()(0,0);
    gps_observation(1,0)=gps_tracker.gps_pose.translation()(1,0);
    gps_observation(5,0)=gps_tracker.gps_pose.translation()(1,0);


    Eigen::Matrix<double, 6, 6> temp=Eigen::Matrix<double, 6, 6>::Zero();
  cout<<temp(0,0)<<" "<<temp(1,0)<<" "<<temp(2,0)<<" "
  <<temp(0,1)<<" "<<temp(1,1)<<" "<<temp(2,1)<<" "
  <<temp(0,2)<<" "<<temp(1,2)<<" "<<temp(2,2)<<" "
  <<temp(0,3)<<" "<<temp(1,3)<<" "<<temp(2,3)<<endl;
    temp(0,0)=gps_tracker.gps_covariance.translation()(0,0);
    temp(1,1)=gps_tracker.gps_covariance.translation()(1,0);
    temp(2,2)=covariance(2,2);
    temp(3,3)=covariance(3,3);
    temp(4,4)=covariance(4,4);
    temp(5,5)=covariance(5,5);
    GaussianDistribution<double, 6> delta(
      Eigen::Matrix<double, 6, 1>::Zero(), temp);

    kalman_filter_.Observe<6>(
      [this, &pose](const State& state) -> Eigen::Matrix<double, 6, 1> {
        const transform::Rigid3d state_pose = RigidFromState(state);
        const Eigen::Vector3d delta_orientation =
            transform::RotationQuaternionToAngleAxisVector(
                pose.rotation().inverse() * state_pose.rotation());
        const Eigen::Vector3d delta_translation =
            state_pose.translation() - pose.translation();
        Eigen::Matrix<double, 6, 1> return_value;
        return_value << delta_translation, delta_orientation;
        return return_value;
      },
      delta,gps_observation);
  }
  else

  {
   //   cout<<"gps_data is not ok"<<endl;
     GaussianDistribution<double, 6> delta(
      Eigen::Matrix<double, 6, 1>::Zero(), covariance);
     kalman_filter_.Observe<6>(
      [this, &pose](const State& state) -> Eigen::Matrix<double, 6, 1> {
        const transform::Rigid3d state_pose = RigidFromState(state);
        const Eigen::Vector3d delta_orientation =
            transform::RotationQuaternionToAngleAxisVector(
                pose.rotation().inverse() * state_pose.rotation());
        const Eigen::Vector3d delta_translation =
            state_pose.translation() - pose.translation();
        Eigen::Matrix<double, 6, 1> return_value;
        return_value << delta_translation, delta_orientation;
        return return_value;
      },
      delta,gps_observation);
  }

}

// Updates from the odometer are in the odometer's map-like frame, called the
// 'odometry' frame. The odometer_pose converts data from the map frame
// into the odometry frame.
void PoseTracker::AddOdometerPoseObservation(
    const common::Time time, const transform::Rigid3d& odometer_pose,
    const PoseCovariance& covariance) {
  if (!odometry_state_tracker_.empty()) {
    const auto& previous_odometry_state = odometry_state_tracker_.newest();
    const transform::Rigid3d delta =
        previous_odometry_state.odometer_pose.inverse() * odometer_pose;
    const transform::Rigid3d new_pose =
        previous_odometry_state.state_pose * delta;
    AddPoseObservation(time, new_pose, covariance);
  }

  const Distribution belief = GetBelief(time);

  odometry_state_tracker_.AddOdometryState(
      {time, odometer_pose, RigidFromState(belief.GetMean())});
}

const OdometryStateTracker::OdometryStates& PoseTracker::odometry_states()
    const {
  return odometry_state_tracker_.odometry_states();
}

transform::Rigid3d PoseTracker::RigidFromState(
    const PoseTracker::State& state) {
  return transform::Rigid3d(
      Eigen::Vector3d(state[PoseTracker::kMapPositionX],
                      state[PoseTracker::kMapPositionY],
                      state[PoseTracker::kMapPositionZ]),
      transform::AngleAxisVectorToRotationQuaternion(
          Eigen::Vector3d(state[PoseTracker::kMapOrientationX],
                          state[PoseTracker::kMapOrientationY],
                          state[PoseTracker::kMapOrientationZ])) *
          imu_tracker_.orientation());
}

Pose2DCovariance Project2D(const PoseCovariance& covariance) {
  Pose2DCovariance projected_covariance;
  projected_covariance.block<2, 2>(0, 0) = covariance.block<2, 2>(0, 0);
  projected_covariance.block<2, 1>(0, 2) = covariance.block<2, 1>(0, 5);
  projected_covariance.block<1, 2>(2, 0) = covariance.block<1, 2>(5, 0);
  projected_covariance(2, 2) = covariance(5, 5);
  return projected_covariance;
}

PoseCovariance Embed3D(const Pose2DCovariance& embedded_covariance,
                       const double position_variance,
                       const double orientation_variance) {
  PoseCovariance covariance;
  covariance.setZero();

  cout<<covariance(0,0)<<" "<<covariance(1,0)<<" "<<covariance(2,0)<<" "
  <<covariance(0,1)<<" "<<covariance(1,1)<<" "<<covariance(2,1)<<" "
  <<covariance(0,2)<<" "<<covariance(1,2)<<" "<<covariance(2,2)<<" "
  <<covariance(0,3)<<" "<<covariance(1,3)<<" "<<covariance(2,3)<<endl;

  covariance.block<2, 2>(0, 0) = embedded_covariance.block<2, 2>(0, 0);
  covariance.block<2, 1>(0, 5) = embedded_covariance.block<2, 1>(0, 2);
  covariance.block<1, 2>(5, 0) = embedded_covariance.block<1, 2>(2, 0);
  covariance(5, 5) = embedded_covariance(2, 2);

  covariance(2, 2) = position_variance;
  covariance(3, 3) = orientation_variance;
  covariance(4, 4) = orientation_variance;
  return covariance;
}

PoseCovariance PoseCovarianceFromProtoMatrix(
    const sensor::proto::Matrix& proto_matrix) {
  PoseCovariance covariance;
  CHECK_EQ(proto_matrix.rows(), 6);
  CHECK_EQ(proto_matrix.cols(), 6);
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      covariance(i, j) = proto_matrix.data(i * 6 + j);
    }
  }
  return covariance;
}

Gps_tracker PoseTracker:: GetGpsPose()
{
    return gps_tracker;
}

void  PoseTracker::SetGpsPose(int64 timestamp, const cartographer::transform::Rigid2d& gps_data,
    const cartographer::transform::Rigid2d& gps_data_covariance,int32 gps_flag)
{
  //  LOG(INFO) <<"pose tracker gps data time >>>>>"<<timestamp<<endl;
    gps_tracker.time=timestamp;
    gps_tracker.gps_pose=gps_data;
    gps_tracker.gps_covariance=gps_data_covariance;
    gps_tracker.gps_flag=gps_flag;
    count_pose++;
    if(count_pose==80)
   {
       cout<<"gps : "<<gps_data.translation()(0,0)+9.1411<<"  "<<gps_data.translation()(1,0)+60.6891<<endl;
       cout<<"kalman : "<<kalman_filter_.GetBelief().GetMean()(0,0)<<"  "<<kalman_filter_.GetBelief().GetMean()(1,0)<<endl;
       count_pose=0;
   }
}
}  // namespace kalman_filter
}  // namespace cartographer
