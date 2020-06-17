/*
 * hunter_sim_params.hpp
 *
 * Created on: Sep 27, 2019 15:08
 * Description:
 *
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef SCOUT_SIM_PARAMS_HPP
#define SCOUT_SIM_PARAMS_HPP

#include <cstdint>

namespace westonrobot {
/* hunter Parameters */
struct HunterSimParams {
  static constexpr double track =
      0.576;  // in meter (left & right wheel distance)
  static constexpr double wheelbase =
      0.648;  // in meter (front & rear wheel distance)
  static constexpr double wheel_radius = 0.165;  // in meter

  // from user manual v1.2.6_S P4
  // max linear velocity: 1.5 m/s
  static constexpr double max_steer_angle = 0.65;  // in rad, 0.75 for inner wheel
  static constexpr double max_linear_speed = 1.5;  // in m/ss
};
}  // namespace westonrobot

#endif /* SCOUT_SIM_PARAMS_HPP */
