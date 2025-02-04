#include "madgwick_filter.h"

#include <iostream>

namespace madgwick_filter {

MadgwickFilter::MadgwickFilter(const Parameters& parameters)
    : parameters_(parameters) {}

MadgwickFilter::~MadgwickFilter() {}

void MadgwickFilter::Update(const bridge::Imu& bridge_imu) {
  constexpr double kGravitationalAcceleration{9.81};
  const auto imu = ConvertFromBridge(bridge_imu);

  if (!condition_.is_bias_initialized) {
    imu_queue_.push_back(imu);
    if (imu_queue_.size() > 100) {
      condition_.is_bias_initialized = InitializeBias();
    }
    return;
  }

  const auto wm = imu.angular_velocity - gyro_bias_;
  // const auto wm = imu.angular_velocity;
  Quaternion o_w;
  o_w.w = 0.0;
  o_w.x = wm.x();
  o_w.y = wm.y();
  o_w.z = wm.z();

  // Propagation via angular velocity
  if (prev_time_ == 0.0) {
    prev_time_ = imu.time;
    return;
  }
  const double dt = imu.time - prev_time_;

  Orientation a{Orientation::Identity()};
  Orientation b;
  b.w() = 0.0;
  b.x() = wm.x();
  b.y() = wm.y();
  b.z() = wm.z();

  Orientation c = a * b;
  std::cerr << "c: " << c.coeffs().transpose() << std::endl;

  std::cerr << "wm: " << wm.transpose() << std::endl;

  std::cerr << "dt: " << dt << std::endl;
  Quaternion dq_w;
  dq_w = (orientation_ * o_w);
  // dq_w *= 0.5;
  std::cerr << "orientation_: " << orientation_.w << " " << orientation_.x
            << " " << orientation_.y << " " << orientation_.z << std::endl;
  std::cerr << "dq_w: " << dq_w.w << " " << dq_w.x << " " << dq_w.y << " "
            << dq_w.z << std::endl;

  Quaternion q_w;
  q_w = (orientation_ + (dq_w * dt));
  q_w.normalize();

  // Compensate orientation using linear acceleration only when the norm
  // of acceleration is near the gravity.
  const auto am = imu.linear_acceleration - acc_bias_;
  bool update_by_acceleration = false;
  if (std::abs(am.norm() - kGravitationalAcceleration) < 1e-1)
    update_by_acceleration = true;

  Quaternion q_a;
  q_a.w = 0.0;
  q_a.x = am.x();
  q_a.y = am.y();
  q_a.z = am.z();

  Quaternion gradient;

  orientation_ = q_w;

  std::cerr << orientation_.w << " " << q_w.w << std::endl;

  prev_time_ = imu.time;
}

bridge::Orientation MadgwickFilter::GetOrientation() const {
  bridge::Orientation bridge_orientation;
  bridge_orientation.w = orientation_.w;
  bridge_orientation.x = orientation_.x;
  bridge_orientation.y = orientation_.y;
  bridge_orientation.z = orientation_.z;
  return bridge_orientation;
}

bool MadgwickFilter::InitializeBias() {
  // TODO(@): estimate acc bias
  Vec3 gyro_bias{Vec3::Zero()};
  for (const auto& imu : imu_queue_) {
    gyro_bias += imu.angular_velocity;
  }
  gyro_bias /= static_cast<double>(imu_queue_.size());
  std::cerr << "initialization OK! gyro bias: " << gyro_bias.transpose()
            << "\n";
  gyro_bias_ = gyro_bias;
  return true;
}

Imu MadgwickFilter::ConvertFromBridge(const bridge::Imu& bridge_imu) {
  Imu imu;
  imu.time = bridge_imu.time;
  imu.linear_acceleration.x() = bridge_imu.linear_acceleration.x;
  imu.linear_acceleration.y() = bridge_imu.linear_acceleration.y;
  imu.linear_acceleration.z() = bridge_imu.linear_acceleration.z;
  imu.angular_velocity.x() = bridge_imu.angular_velocity.x;
  imu.angular_velocity.y() = bridge_imu.angular_velocity.y;
  imu.angular_velocity.z() = bridge_imu.angular_velocity.z;
  return imu;
}

}  // namespace madgwick_filter