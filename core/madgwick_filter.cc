#include "madgwick_filter.h"

#include <iostream>

namespace madgwick_filter {

MadgwickFilter::MadgwickFilter(const Parameters& parameters)
    : parameters_(parameters) {}

MadgwickFilter::~MadgwickFilter() {}

void MadgwickFilter::Update(const bridge::Imu& imu_data) {
  const double time = imu_data.time;
  const Vec3 linear_acceleration(imu_data.linear_acceleration.x,
                                 imu_data.linear_acceleration.y,
                                 imu_data.linear_acceleration.z);
  const Vec3 angular_velocity(imu_data.angular_velocity.x,
                              imu_data.angular_velocity.y,
                              imu_data.angular_velocity.z);
  const auto am = linear_acceleration - acc_bias_;
  Orientation q_a;
  q_a.w() = 0.0;
  q_a.x() = am.x();
  q_a.y() = am.y();
  q_a.z() = am.z();

  const auto wm = angular_velocity - gyro_bias_;
  Orientation q_w;
  q_w.w() = 0.0;
  q_w.x() = wm.x();
  q_w.y() = wm.y();
  q_w.z() = wm.z();

  std::cerr << "qa: " << q_a.coeffs().transpose() << " "
            << q_w.coeffs().transpose() << std::endl;

  Orientation gradient{Orientation::Identity()};
}

}  // namespace madgwick_filter