#ifndef MADGWICK_FILTER_H_
#define MADGWICK_FILTER_H_

#include <iostream>
#include <queue>

#include "bridge.h"
#include "eigen3/Eigen/Dense"

namespace madgwick_filter {

using Vec3 = Eigen::Matrix<double, 3, 1>;
using Vec4 = Eigen::Matrix<double, 4, 1>;
using Mat3x3 = Eigen::Matrix<double, 3, 3>;
using Mat3x4 = Eigen::Matrix<double, 3, 4>;
using Orientation = Eigen::Quaterniond;

struct Quaternion {
  double w{1.0};
  double x{0.0};
  double y{0.0};
  double z{0.0};

  Quaternion() : w(1.0), x(0.0), y(0.0), z(0.0) {}
  Quaternion(const double _w, const double _x, const double _y, const double _z)
      : w(_w), x(_x), y(_y), z(_z) {}
  Quaternion(const Quaternion& rhs) : w(rhs.w), x(rhs.x), y(rhs.y), z(rhs.z) {}
  Quaternion& operator=(const Quaternion& rhs) {
    w = rhs.w;
    x = rhs.x;
    y = rhs.y;
    z = rhs.z;
    return *this;
  }
  Quaternion inverse() {
    Quaternion inv_q(w, -x, -y, -z);
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
    Quaternion res(w + rhs.w, x + rhs.x, y + rhs.y, z + rhs.z);
    return res;
  }
  Quaternion operator-(const Quaternion& rhs) const {
    Quaternion res(w - rhs.w, x - rhs.x, y - rhs.y, z - rhs.z);
    return res;
  }
  Quaternion operator*(const Quaternion& rhs) const {
    Quaternion res;
    res.w = w * rhs.w - x * rhs.x - y * rhs.y - z * rhs.z;
    res.x = w * rhs.x + x * rhs.w + y * rhs.z - z * rhs.y;
    res.y = w * rhs.y - x * rhs.z + y * rhs.w + z * rhs.x;
    res.z = w * rhs.z + x * rhs.y - y * rhs.x + z * rhs.w;
    return res;
  }
  Quaternion operator*(const double scalar) const {
    Quaternion res(w * scalar, x * scalar, y * scalar, z * scalar);
    return res;
  }
  Quaternion& operator+=(const Quaternion& rhs) {
    *this = *this + rhs;
    return *this;
  }
  Quaternion& operator-=(const Quaternion& rhs) {
    *this = *this - rhs;
    return *this;
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
  Quaternion orientation_;

  Vec3 acc_bias_{Vec3::Zero()};
  Vec3 gyro_bias_{Vec3::Zero()};
  Vec3 mag_bias_{Vec3::Zero()};

  double prev_time_{0.0};
};

}  // namespace madgwick_filter

#endif  // MADGWICK_FILTER_H_