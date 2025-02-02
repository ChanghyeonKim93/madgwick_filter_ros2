#include "madgwick_filter.h"

namespace madgwick_filter {

MadgwickFilter::MadgwickFilter(const Parameters& parameters)
    : parameters_(parameters) {}

MadgwickFilter::~MadgwickFilter() {}

void MadgwickFilter::Update(const Vec3& linear_acceleration,
                            const Vec3& angular_velocity) {
  const auto a = linear_acceleration - acc_bias_;
  Orientation q_a;
  q_a.w() = 0.0;
  q_a.x() = a.x();
  q_a.y() = a.y();
  q_a.z() = a.z();

  const auto w = angular_velocity - gyro_bias_;
  Orientation q_w;
  q_w.w() = 0.0;
  q_w.x() = w.x();
  q_w.y() = w.y();
  q_w.z() = w.z();

    Orientation gradient{Orientation::Identity()};
}

}  // namespace madgwick_filter