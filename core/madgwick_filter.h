#ifndef MADGWICK_FILTER_H_
#define MADGWICK_FILTER_H_

#include <queue>

#include "bridge.h"
#include "eigen3/Eigen/Dense"

namespace madgwick_filter {

using Vec3 = Eigen::Vector3d;
using Mat3x3 = Eigen::Matrix3d;
using Orientation = Eigen::Quaterniond;

struct Quaternion {
  double w{1.0};
  double x{0.0};
  double y{0.0};
  double z{0.0};

  Quaternion inverse() {
    Quaternion inv_q;
    inv_q.w = w;
    inv_q.x = -x;
    inv_q.y = -y;
    inv_q.z = -z;
    return inv_q;
  }
  void normalize() {
    const double norm = std::sqrt(w * w + x * x + y * y + z * z);
    const double inv_norm = 1.0 / norm;
    w *= inv_norm;
    x *= inv_norm;
    y *= inv_norm;
    z *= inv_norm;
  }
  Quaternion operator+(const Quaternion& rhs) const {
    Quaternion res;
    res.w = w + rhs.w;
    res.x = x + rhs.x;
    res.y = y + rhs.y;
    res.z = z + rhs.z;
    return res;
  }
  Quaternion& operator+=(const Quaternion& rhs) {
    w += rhs.w;
    x += rhs.x;
    y += rhs.y;
    z += rhs.z;
    return *this;
  }
  Quaternion operator*(const Quaternion& rhs) const {
    Quaternion res;
    res.w = w * rhs.w - x * rhs.x - y * rhs.y - z * rhs.z;
    res.x = w * rhs.x + x * rhs.w + y * rhs.z - z * rhs.y;
    res.y = w * rhs.y - x * rhs.z + y * rhs.w + z * rhs.x;
    res.z = w * rhs.z + x * rhs.y - y * rhs.x + z * rhs.w;
    res.normalize();
    return res;
  }
  Quaternion operator*(const double scalar) const {
    Quaternion res;
    res.w = w * scalar;
    res.x = x * scalar;
    res.y = y * scalar;
    res.z = z * scalar;
    return res;
  }
  Quaternion& operator*=(const Quaternion& rhs) {
    double w_new = w * rhs.w - x * rhs.x - y * rhs.y - z * rhs.z;
    double x_new = w * rhs.x + x * rhs.w + y * rhs.z - z * rhs.y;
    double y_new = w * rhs.y - x * rhs.z + y * rhs.w + z * rhs.x;
    double z_new = w * rhs.z + x * rhs.y - y * rhs.x + z * rhs.w;
    w = w_new;
    x = x_new;
    y = y_new;
    z = z_new;
    return *this;
  }
  Quaternion& operator*=(const double scalar) {
    w *= scalar;
    x *= scalar;
    y *= scalar;
    z *= scalar;
    return *this;
  }
};

struct Parameters {};

struct Condition {
  bool is_bias_initialized{false};
};

struct Imu {
  double time{0.0};
  Vec3 linear_acceleration{Vec3::Zero()};
  Vec3 angular_velocity{Vec3::Zero()};
};

class MadgwickFilter {
 public:
  explicit MadgwickFilter(const Parameters& parameters);

  ~MadgwickFilter();

  void Update(const bridge::Imu& imu_data);

  bridge::Orientation GetOrientation() const;

 private:
  bool InitializeBias();
  Imu ConvertFromBridge(const bridge::Imu& bridge_imu);

  std::deque<Imu> imu_queue_;

  const Parameters parameters_;
  Condition condition_;

  Vec3 acc_bias_{Vec3::Zero()};
  Vec3 gyro_bias_{Vec3::Zero()};

  double prev_time_{0.0};
  Quaternion orientation_;
};

}  // namespace madgwick_filter

#endif  // MADGWICK_FILTER_H_