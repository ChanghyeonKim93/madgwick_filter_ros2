#ifndef MADGWICK_FILTER_H_
#define MADGWICK_FILTER_H_

#include <iostream>
#include <queue>

#include "eigen3/Eigen/Dense"

#include "bridge.h"
#include "types.h"

namespace madgwick_filter {

struct Parameters {
  bool use_magnetometer{false};
  double gravitational_acceleration{9.81};  // [m/s2]
  struct {
    double linear_acceleration{0.2};
    double magnetometer{0.001};
  } convergence_rate;
  struct {
    double gravitation_diff_threshold{1e-2};
  } update_criteria;
  struct {
    int num_data_for_gyro_bias{100};
    int num_data_for_initial_pose{100};
  } initialization;
};

struct Condition {
  bool is_bias_initialized{false};
  bool is_initial_pose_estimated{false};
};

class MadgwickFilter {
 public:
  explicit MadgwickFilter(const Parameters& parameters);

  ~MadgwickFilter();

  void Update(const bridge::Imu& imu_data);

  void SetImu(const bridge::Imu& bridge_imu);
  void SetMagneticField(const bridge::Vec3& bridge_magnetic_field);

  void UpdateByMagnetometer(const bridge::Vec3& magnetic_field);

  bridge::Orientation GetOrientation() const;

 private:
  bool InitializeGyroBias();
  Quaternion ComputeGradientByAngularVelocity(const Vec3& angular_velocity);
  Quaternion ComputeGradientlByLinearAcceleration(
      const Vec3& linear_acceleration);
  Quaternion ComputeGradientlByMagnetometer(const Vec3& magnetic_field);

  Imu ConvertFromBridge(const bridge::Imu& bridge_imu);

  const Parameters parameters_;

  Condition condition_;
  std::deque<Imu> imu_queue_;
  std::deque<Vec3> magnetic_field_queue_;

  Quaternion orientation_;

  Vec3 acc_bias_{Vec3::Zero()};
  Vec3 gyro_bias_{Vec3::Zero()};
  Vec3 mag_bias_{Vec3::Zero()};

  double previous_time_{0.0};
};

}  // namespace madgwick_filter

#endif  // MADGWICK_FILTER_H_