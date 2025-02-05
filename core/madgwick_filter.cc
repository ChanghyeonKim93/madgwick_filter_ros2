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
  Quaternion dq_w;
  dq_w = (orientation_ * o_w) * 0.5;

  Quaternion q_w;
  q_w = (orientation_ + (dq_w * dt));
  q_w.normalize();

  // Compensate orientation using linear acceleration only when the norm
  // of acceleration is near the gravity.
  const auto am = imu.linear_acceleration - acc_bias_;
  bool update_by_acceleration = false;
  if (std::abs(am.norm() - kGravitationalAcceleration) < 1e-1)
    update_by_acceleration = true;

  if (update_by_acceleration) {
    std::cerr << "UPDATE Acc\n";
    Quaternion q_a;
    q_a.w = 0.0;
    q_a.x = am.x();
    q_a.y = am.y();
    q_a.z = am.z();

    const auto& q = orientation_;
    Vec3 fg;
    fg(0) = 2.0 * (q.x * q.z - q.w * q.y) - am.x() / 9.81;
    fg(1) = 2.0 * (q.w * q.x + q.y * q.z) - am.y() / 9.81;
    fg(2) = 2.0 * (0.5 - q.x * q.x - q.y * q.y) - am.z() / 9.81;
    Eigen::Matrix<double, 3, 4> Jg;
    Jg << -2.0 * q.y, 2.0 * q.z, -2.0 * q.w, 2.0 * q.x,  //
        2.0 * q.x, 2.0 * q.w, 2.0 * q.z, 2.0 * q.y,      ///
        0.0, -4.0 * q.x, -4.0 * q.y, 0.0;
    Eigen::Vector4d del_f = Jg.transpose() * fg;
    std::cerr << "fg: " << fg.transpose() << std::endl;
    std::cerr << "del_f: " << del_f.transpose() << std::endl;
    Quaternion dq_a(del_f.w(), del_f.x(), del_f.y(), del_f.z());
    dq_a *= 1.0 / del_f.norm();

    constexpr double kNoiseLevel{1e-1};
    const double kBeta = kNoiseLevel * std::sqrt(3.0 / 4.0);
    Quaternion dq_est = dq_w + dq_a * (-kBeta);
    orientation_ = orientation_ + (dq_est * dt);

    // orientation_ = orientation_ + (dq_w * dt);
    // orientation_.x += (-kBeta * dq_a.x * dt);
    // orientation_.y += (-kBeta * dq_a.y * dt);
  } else {
    orientation_ = orientation_ + (dq_w * dt);
  }
  orientation_.normalize();

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