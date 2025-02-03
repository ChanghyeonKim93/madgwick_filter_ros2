#ifndef MADGWICK_FILTER_H_
#define MADGWICK_FILTER_H_

#include "bridge.h"
#include "eigen3/Eigen/Dense"

namespace madgwick_filter {

using Vec3 = Eigen::Vector3d;
using Mat3x3 = Eigen::Matrix3d;
using Orientation = Eigen::Quaterniond;

struct Parameters {};

class MadgwickFilter {
 public:
  explicit MadgwickFilter(const Parameters& parameters);

  ~MadgwickFilter();

  void Update(const bridge::Imu& imu_data);

 private:
  const Parameters parameters_;

  Vec3 acc_bias_{Vec3::Zero()};
  Vec3 gyro_bias_{Vec3::Zero()};

  Orientation orientation_{Orientation::Identity()};
};

}  // namespace madgwick_filter

#endif  // MADGWICK_FILTER_H_