#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif

#ifndef _SIM_UTILS_CPP_
#define _SIM_UTILS_CPP_

#include "Simulation.h"
#include "PointVectors.h"

DaN Simulation::dan_to_plane(
    const Vec3D &point,
    const Vec3D &point_on_plane,
    const Vec3D &plane_normal) {
  return {
    plane_normal.dot(point - point_on_plane),
    plane_normal};
}

DaN Simulation::dan_to_sphere_inner(
    const Vec3D &point,
    const Vec3D &sphere_center,
    const double &sphere_radius) {
  return {
    sphere_radius - (point - sphere_center).len(),
    (sphere_center - point).normalize()};
}

DaN Simulation::dan_to_sphere_outer(
    const Vec3D &point,
    const Vec3D &sphere_center,
    const double &sphere_radius) {
  return {
    (point - sphere_center).len() - sphere_radius,
    (point - sphere_center).normalize()};
}

DaN Simulation::dan_to_arena_quarter(const Vec3D &point) {
  // Ground
  DaN dan = dan_to_plane(point, Vec3D(0.0, 0.0, 0.0), Vec3D(0.0, 0.0, 1.0));

  // Ceiling
  dan = std::min(dan, dan_to_plane(point, Vec3D(0, 0, arena.height), Vec3D(0, 0, -1)));

  // Side x
  dan = std::min(dan, dan_to_plane(point, Vec3D(arena.width/2.0, 0, 0), Vec3D(-1, 0, 0)));

  // Side z (back of goal)
  dan = std::min(dan, dan_to_plane(point, Vec3D(0, arena.depth/2.0 + arena.goal_depth, 0), Vec3D(0, -1, 0)));

  // Side z
  Vec2D v = Vec2D(point.x, point.y) - Vec2D(
    (arena.goal_width / 2.0 - arena.goal_top_radius),
    (arena.goal_height - arena.goal_top_radius)
  );
  if (point.x >= arena.goal_width / 2.0 + arena.goal_side_radius or
      point.y >= arena.goal_height + arena.goal_side_radius or (
        v.x > 0 and
        v.z > 0 and
        v.len() >= arena.goal_top_radius + arena.goal_side_radius)){
    dan = std::min(dan, dan_to_plane(point, Vec3D(0, arena.depth/2.0, 0), Vec3D(0, -1, 0)));
  }

  // Side x & ceiling (goal)
  if (point.z >= arena.depth / 2.0 + arena.goal_side_radius) {
    // x
    dan = std::min(dan, dan_to_plane(point, Vec3D(arena.goal_width/2.0, 0, 0), Vec3D(-1, 0, 0)));
    // y
    dan = std::min(dan, dan_to_plane(point, Vec3D(0, 0, arena.goal_height), Vec3D(0, 0, -1)));
  }

  // goal back corners
  if (point.z > arena.depth/2.0 + arena.goal_depth - arena.bottom_radius) {
    dan = std::min(
      dan,
      dan_to_sphere_inner(
        point,
        Vec3D(
          clamp(point.x, arena.bottom_radius - arena.goal_width/2.0, arena.goal_width/2.0 - arena.bottom_radius),
          (arena.depth / 2.0) + arena.goal_depth - arena.bottom_radius,
          clamp(point.y, arena.bottom_radius, arena.goal_height - arena.goal_top_radius)
        ),
        arena.bottom_radius
      )
    );
  }

  // Corner
  if (point.x > arena.width/2.0 - arena.corner_radius and
      point.z > arena.depth/2.0 - arena.corner_radius) {
    dan = std::min(
      dan,
      dan_to_sphere_inner(
        point,
        Vec3D(
          arena.width/2.0 - arena.corner_radius,
          arena.depth/2.0 - arena.corner_radius,
          point.y
        ),
        arena.corner_radius
      )
    );
  }

  // Goal outer corner
  if (point.z < arena.depth/2.0 + arena.goal_side_radius) {
    // Side x
    if (point.x < arena.goal_width/2.0 + arena.goal_side_radius) {
      dan = std::min(
        dan,
        dan_to_sphere_outer(
          point,
          Vec3D(
            arena.goal_width/2.0 + arena.goal_side_radius,
            arena.depth/2.0 + arena.goal_side_radius,
            point.y
          ),
          arena.goal_side_radius
        )
      );
    }
    // Ceiling
    if (point.y < arena.goal_height + arena.goal_side_radius) {
      dan = std::min(
        dan,
        dan_to_sphere_outer(
          point,
          Vec3D(
            point.x,
            arena.depth/2.0 + arena.goal_side_radius,
            arena.goal_height + arena.goal_side_radius
          ),
          arena.goal_side_radius
        )
      );
    }
    // Top corner
    Vec2D o = {
      arena.goal_width/2.0 - arena.goal_top_radius,
      arena.goal_height - arena.goal_top_radius
    };
    v = Vec2D(point.x, point.y) - o;
    if (v.x > 0 and v.z > 0) {
      o += v.normalize() * (arena.goal_top_radius + arena.goal_side_radius);
      dan = std::min(
        dan,
        dan_to_sphere_outer(
          point,
          Vec3D(
            o.x,
            arena.depth/2.0 + arena.goal_side_radius,
            o.z
          ),
          arena.goal_side_radius
        )
      );
    }
  }

  // Goal inside top corners
  if (point.z > arena.depth/2.0 + arena.goal_side_radius and
      point.y > arena.goal_height - arena.goal_top_radius) {
    // Side x
    if (point.x > arena.goal_width/2.0 - arena.goal_top_radius) {
      dan = std::min(
        dan,
        dan_to_sphere_inner(
          point,
          Vec3D(
            arena.goal_width/2.0 - arena.goal_top_radius,
            point.z,
            arena.goal_height - arena.goal_top_radius
          ),
          arena.goal_top_radius
        )
      );
    }
    // Side z
    if (point.z > arena.depth/2.0 + arena.goal_depth - arena.goal_top_radius) {
      dan = std::min(
        dan,
        dan_to_sphere_inner(
          point,
          Vec3D(
            point.x,
            arena.depth/2 + arena.goal_depth - arena.goal_top_radius,
            arena.goal_height - arena.goal_top_radius
          ),
          arena.goal_top_radius
        )
      );
    }
  }

  // Bottom corners
  if (point.y < arena.bottom_radius) {
    // Side x
    if (point.x > arena.width/2.0 - arena.bottom_radius) {
      dan = std::min(
        dan,
        dan_to_sphere_inner(
          point,
          Vec3D(
            arena.width/2 - arena.bottom_radius,
            point.z,
            arena.bottom_radius
          ),
          arena.bottom_radius
        )
      );
    }
    // Side z
    if (point.z > arena.depth/2.0 - arena.bottom_radius and
        point.x >= arena.goal_width/2.0 + arena.goal_side_radius) {
      dan = std::min(
        dan,
        dan_to_sphere_inner(
          point,
          Vec3D(
            point.x,
            arena.depth/2.0 - arena.bottom_radius,
            arena.bottom_radius
          ),
          arena.bottom_radius
        )
      );
    }
    // Side z (goal)
    if (point.z > arena.depth/2.0 + arena.goal_depth - arena.bottom_radius) {
      dan = std::min(
        dan,
        dan_to_sphere_inner(
          point,
          Vec3D(
            point.x,
            arena.depth/2.0 + arena.goal_depth - arena.bottom_radius,
            arena.bottom_radius
          ),
          arena.bottom_radius
        )
      );
    }
    // Goal outer corner
    Vec2D o = {
      arena.goal_width/2.0 + arena.goal_side_radius,
      arena.depth/2.0 + arena.goal_side_radius
    };
    Vec2D v = Vec2D(point.x, point.z) - o;
    if (v.x < 0 and v.z < 0 and v.len() < arena.goal_side_radius + arena.bottom_radius) {
      o += v.normalize() * (arena.goal_side_radius + arena.bottom_radius);
      dan = std::min(
        dan,
        dan_to_sphere_inner(
          point,
          Vec3D(
            o.x,
            o.z,
            arena.bottom_radius
          ),
          arena.bottom_radius
        )
      );
    }
    // Side x (goal)
    if (point.z >= arena.depth/2.0 + arena.goal_side_radius and
        point.x > arena.goal_width/2.0 - arena.bottom_radius) {
      dan = std::min(
        dan,
        dan_to_sphere_inner(
          point,
          Vec3D(
            arena.goal_width/2.0 - arena.bottom_radius,
            point.z,
            arena.bottom_radius
          ),
          arena.bottom_radius
        )
      );
    }
    // corner
    if (point.x > arena.width/2.0 - arena.corner_radius and
        point.z > arena.depth/2.0 - arena.corner_radius) {
      Vec2D corner_o = {
        arena.width/2.0 - arena.corner_radius,
        arena.depth/2.0 - arena.corner_radius
      };
      Vec2D n = Vec2D(point.x, point.z) - corner_o;
      double dist = n.len();
      if (dist > arena.corner_radius - arena.bottom_radius) {
        n *= 1/dist;
        Vec2D o2 = corner_o + n * (arena.corner_radius - arena.bottom_radius);
        dan = std::min(
          dan,
          dan_to_sphere_inner(
            point,
            Vec3D(
              o2.x,
              o2.z,
              arena.bottom_radius
            ),
            arena.bottom_radius
          )
        );
      }
    }
  }

  // Ceiling corners
  if (point.y > arena.height - arena.top_radius) {
    // Side x
    if (point.x > arena.width/2.0 - arena.top_radius) {
      dan = std::min(
        dan,
        dan_to_sphere_inner(
          point,
          Vec3D(
            arena.width/2.0 - arena.top_radius,
            point.z,
            arena.height - arena.top_radius
          ),
          arena.top_radius
        )
      );
    }
    // Side z
    if (point.z > arena.depth/2.0 - arena.top_radius) {
      dan = std::min(
        dan,
        dan_to_sphere_inner(
          point,
          Vec3D(
            point.x,
            arena.depth/2.0 - arena.top_radius,
            arena.height - arena.top_radius
          ),
          arena.top_radius
        )
      );
    }
    // Corner
    if (point.x > arena.width/2.0 - arena.corner_radius and
        point.z > arena.depth/2.0 - arena.corner_radius) {
      Vec2D corner_o = {
        arena.width/2.0 - arena.corner_radius,
        arena.depth/2.0 - arena.corner_radius
      };
      Vec2D dv = Vec2D(point.x, point.z) - corner_o;
      if (dv.len() > arena.corner_radius - arena.top_radius) {
        Vec2D n = dv.normalize();
        Vec2D o2 = corner_o + n * (arena.corner_radius - arena.top_radius);
        dan = std::min(
          dan,
          dan_to_sphere_inner(
              point,
              Vec3D (
                o2.x,
                o2.z,
                arena.height - arena.top_radius
              ),
              arena.top_radius
          )
        );
      }
    }
  }

  return dan;
}

DaN Simulation::dan_to_arena(Vec3D &point) {
  bool negate_x = point.x < 0;
  bool negate_z = point.z < 0;
  if (negate_x) point.x *= -1;
  if (negate_z) point.z *= -1;
  DaN arena_quarter_collision = dan_to_arena_quarter(point);
  if (negate_x) {
    point.x *= -1;
    arena_quarter_collision.normal.x *= -1;
  }
  if (negate_z) {
    point.z *= -1;
    arena_quarter_collision.normal.z *= -1;
  }
  return arena_quarter_collision;
}

#endif
