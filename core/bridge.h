#ifndef MADGWICK_FILTER_BRIDGE_H_
#define MADGWICK_FILTER_BRIDGE_H_

namespace madgwick_filter {
namespace bridge {

struct Vec3 {
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Orientation {
  double w{1.0};
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Imu {
  double time{0.0};
  Vec3 linear_acceleration;
  Vec3 angular_velocity;
};

}  // namespace bridge
}  // namespace madgwick_filter

#endif  // MADGWICK_FILTER_BRIDGE_H_