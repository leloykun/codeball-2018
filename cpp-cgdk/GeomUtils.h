#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif

#ifndef _GEOM_UTILS_H_
#define _GEOM_UTILS_H_

#include "PointVectors.h"
#include <tuple>

namespace geom {
  std::vector<Vec2D> get_tangents_to_circle(
      const Vec2D &c_center,
      const double &c_radius,
      const Vec2D &point);

  Vec2D get_segment_circle_intersection(
      const Vec2D &c_center,
      const double &c_radius,
      const Vec2D &in_point,
      const Vec2D &out_point);

  std::tuple<bool, Vec2D> ray_segment_intersection(
      const Vec2D &origin,
      const Vec2D &dir,
      const Vec2D &p1,
      const Vec2D &p2);

  Vec2D offset_to(const Vec2D &origin, const Vec2D &to, const double &offset);

  double calc_jump_height(const double &jump_speed, const double &gravity);

  double time_to_go_to(
      const Vec2D &origin,
      const Vec2D &init_velocity,
      const Vec2D &destination,
      const double &acceleration = 100,
      const double &max_speed = 30);
}

#endif
