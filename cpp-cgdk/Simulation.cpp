#include "Simulation.h"
#include "PointVectors.h"
#include <iostream>
#include <random>
#include <vector>

using namespace model;

Simulation::Simulation(const Ball &ball,
                       const std::vector<Robot> &robots,
                       const std::vector<Vec3D> &target_velocities,
                       const std::vector<double> &jump_speeds,
                       const Rules &rules,
                       double delta_time) {
  this->ball = {
    Vec3D(ball.x, ball.z, ball.y),
    Vec3D(ball.velocity_x, ball.velocity_z, ball.velocity_y),
    Vec3D(),
    rules.BALL_RADIUS,
    0.0,
    rules.BALL_MASS,
    rules.BALL_ARENA_E,
    BALL
  };
  this->ball_col = this->ball;
  for (auto robot : robots) {
    this->robots.push_back({
      Vec3D(robot.x, robot.z, robot.y),
      Vec3D(robot.velocity_x, robot.velocity_z, robot.velocity_y),
      target_velocities[robot.id],
      rules.ROBOT_RADIUS,
      jump_speeds[robot.id],
      rules.ROBOT_MASS,
      rules.ROBOT_ARENA_E,
      (robot.is_teammate ? ALLY : ENEMY)
    });
  }
  this->rules = rules;
  this->delta_time = delta_time;
}

double Simulation::clamp(double a, double min_val, double max_val) {
  return std::min(std::max(a, min_val), max_val);
}

Vec3D Simulation::clamp(const Vec3D &v, double val) {
  return {
    std::min(v.x, val),
    std::min(v.z, val),
    std::min(v.y, val),
  };
}

void Simulation::update() {
  move(robots);
  move(ball);
  move(ball_col);

  for (int i = 0; i < int(robots.size()); ++i)
    for (int j = 0; j < i; ++j)
      collide_entities(robots[i], robots[j]);
  for (Entity &robot : robots) {
    // collide_entities(robot, ball);
    collide_entities(robot, ball_col);
    collide_with_arena(robot);
  }
  collide_with_arena(ball);
  collide_with_arena(ball_col);

  sim_tick++;
}

void Simulation::move(Entity &en) {
  en.velocity.clamp(this->rules.MAX_ENTITY_SPEED);
  en.position += en.velocity * this->delta_time;
  en.position.y -= this->rules.GRAVITY * this->delta_time * this->delta_time / 2.0;
  en.velocity.y -= this->rules.GRAVITY * this->delta_time;
}

void Simulation::move(std::vector<Entity> &ens) {
  for (Entity &en : ens) {
    // assuming that these are the robots
    DaN arena_collision = dan_to_arena(en.position);
    double penetration = en.radius - arena_collision.distance;
    if (penetration > 0) {
      // means that the entity is touching the arena
      Vec3D target_velocity = en.target_velocity;
      target_velocity.clamp(rules.ROBOT_MAX_GROUND_SPEED);
      target_velocity -= arena_collision.normal * arena_collision.normal.dot(target_velocity);
      Vec3D target_velocity_change = target_velocity - en.velocity;
      if (target_velocity_change.len() > 0) {
        double acceleration = rules.ROBOT_ACCELERATION * std::max(0.0, arena_collision.normal.y);
        en.velocity += clamp(target_velocity_change.normalize() * acceleration * delta_time, target_velocity_change.len());
      }
    }
    move(en);
    if (penetration >= 0) {
      if(might_jump(en))
        jump(en);
      if (en.last_sim_jump != sim_tick)
        unjump(en);
    }
  }
}

bool Simulation::might_jump(Entity &en) {
  double dist_to_ball = (en.position - ball_col.position).len();
  if (en.type == ENEMY) {
    if (dist_to_ball < rules.BALL_RADIUS + 2*rules.ROBOT_MAX_RADIUS and
        en.position.z > ball_col.position.z)
      return true;
  } else if (en.type == ALLY) {
    if (dist_to_ball < rules.BALL_RADIUS + 2*rules.ROBOT_MAX_RADIUS and
        en.position.z < ball_col.position.z)
      return true;
  }
  return false;
}

void Simulation::jump(Entity &en) {
  en.radius = rules.ROBOT_MAX_RADIUS;
  en.radius_change_speed = rules.ROBOT_MAX_JUMP_SPEED;
  en.last_sim_jump = sim_tick;
}

void Simulation::unjump(Entity &en) {
  en.radius = rules.ROBOT_MIN_RADIUS;
  en.radius_change_speed = 0.0;
}

bool Simulation::collide_entities(Entity &a, Entity &b) {
  std::uniform_real_distribution<double> unif(rules.MIN_HIT_E, rules.MAX_HIT_E);
  std::default_random_engine re;

  Vec3D delta_position = b.position - a.position;
  double distance = delta_position.len();
  double penetration = a.radius + b.radius - distance;
  if (penetration > 0) {
    double k_a = (1/a.mass) / ((1/a.mass) + (1/b.mass));
    double k_b = (1/b.mass) / ((1/a.mass) + (1/b.mass));
    Vec3D normal = delta_position.normalize();
    a.position -= normal * penetration * k_a;
    b.position += normal * penetration * k_b;
    double delta_velocity = (b.velocity - a.velocity).dot(normal) +
                            b.radius_change_speed - a.radius_change_speed;
    if (delta_velocity < 0) {
      Vec3D impulse = normal * (1 + unif(re)) * delta_velocity;
      a.velocity += impulse * k_a;
      b.velocity -= impulse * k_b;
    }
    return true;
  }
  return false;
}

DaN Simulation::collide_with_arena(Entity &en) {
  DaN arena_collision = dan_to_arena(en.position);
  double penetration = en.radius - arena_collision.distance;
  if (penetration > 0) {
    en.position += arena_collision.normal * penetration;
    double velocity = en.velocity.dot(arena_collision.normal) - en.radius_change_speed;
    if (velocity < 0)
      en.velocity -= arena_collision.normal * (1 + en.coeff_restitution_arena) * velocity;
  }
  return arena_collision;
}

DaN Simulation::dan_to_plane(const Vec3D &point, const Vec3D &point_on_plane, const Vec3D &plane_normal) {
  return {
    plane_normal.dot(point - point_on_plane),
    plane_normal
  };
}

DaN Simulation::dan_to_sphere_inner(const Vec3D &point, const Vec3D &sphere_center, double sphere_radius) {
  return {
    sphere_radius - (point - sphere_center).len(),
    (sphere_center - point).normalize()
  };
}

DaN Simulation::dan_to_sphere_outer(const Vec3D &point, const Vec3D &sphere_center, double sphere_radius) {
  return {
    (point - sphere_center).len() - sphere_radius,
    (point - sphere_center).normalize()
  };
}

DaN Simulation::dan_to_arena_quarter(const Vec3D &point) {
  // Ground
  DaN dan = dan_to_plane(point, Vec3D(0.0, 0.0, 0.0), Vec3D(0.0, 0.0, 1.0));

  // Ceiling
  dan = std::min(dan, dan_to_plane(point, Vec3D(0, 0, rules.arena.height), Vec3D(0, 0, -1)));

  // Side x
  dan = std::min(dan, dan_to_plane(point, Vec3D(rules.arena.width/2.0, 0, 0), Vec3D(-1, 0, 0)));

  // Side z (back of goal)
  dan = std::min(dan, dan_to_plane(point, Vec3D(0, rules.arena.depth/2.0 + rules.arena.goal_depth, 0), Vec3D(0, -1, 0)));

  // Side z
  Vec2D v = Vec2D(point.x, point.y) - Vec2D(
    (rules.arena.goal_width / 2.0 - rules.arena.goal_top_radius),
    (rules.arena.goal_height / 2.0 - rules.arena.goal_top_radius)
  );
  if (point.x >= rules.arena.goal_width / 2.0 + rules.arena.goal_side_radius or
      point.y >= rules.arena.goal_height + rules.arena.goal_side_radius or (
        v.x > 0 and
        v.z > 0 and
        v.len() >= rules.arena.goal_top_radius + rules.arena.goal_side_radius)){
    dan = std::min(dan, dan_to_plane(point, Vec3D(0, rules.arena.depth/2.0, 0), Vec3D(0, -1, 0)));
  }

  // Side x & ceiling (goal)
  if (point.z >= (rules.arena.depth / 2.0) + rules.arena.goal_side_radius) {
    // x
    dan = std::min(dan, dan_to_plane(point, Vec3D(rules.arena.goal_width/2.0, 0, 0), Vec3D(-1, 0, 0)));
    // y
    dan = std::min(dan, dan_to_plane(point, Vec3D(0, 0, rules.arena.goal_height), Vec3D(0, 0, -1)));
  }

  // goal back corners
  if (point.z > rules.arena.depth/2.0 + rules.arena.goal_depth - rules.arena.bottom_radius) {
    dan = std::min(
      dan,
      dan_to_sphere_inner(
        point,
        Vec3D(
          clamp(point.x, rules.arena.bottom_radius - rules.arena.goal_width/2.0, rules.arena.goal_width/2.0 - rules.arena.bottom_radius),
          (rules.arena.depth / 2.0) + rules.arena.goal_depth - rules.arena.bottom_radius,
          clamp(point.y, rules.arena.bottom_radius, rules.arena.goal_height - rules.arena.goal_top_radius)
        ),
        rules.arena.bottom_radius
      )
    );
  }

  // Corner
  if (point.x > rules.arena.width/2.0 - rules.arena.corner_radius and
      point.z > rules.arena.depth/2.0 - rules.arena.corner_radius) {
    dan = std::min(
      dan,
      dan_to_sphere_inner(
        point,
        Vec3D(
          rules.arena.width/2.0 - rules.arena.corner_radius,
          rules.arena.depth/2.0 - rules.arena.corner_radius,
          point.y
        ),
        rules.arena.corner_radius
      )
    );
  }

  // Goal outer corner
  if (point.z < rules.arena.depth/2.0 + rules.arena.goal_side_radius) {
    // Side x
    if (point.x < rules.arena.goal_width/2.0 + rules.arena.goal_side_radius) {
      dan = std::min(
        dan,
        dan_to_sphere_outer(
          point,
          Vec3D(
            rules.arena.goal_width/2.0 + rules.arena.goal_side_radius,
            rules.arena.depth/2.0 + rules.arena.goal_side_radius,
            point.y
          ),
          rules.arena.goal_side_radius
        )
      );
    }
    // Ceiling
    if (point.y < rules.arena.goal_height + rules.arena.goal_side_radius) {
      dan = std::min(
        dan,
        dan_to_sphere_outer(
          point,
          Vec3D(
            point.x,
            rules.arena.depth/2.0 + rules.arena.goal_side_radius,
            rules.arena.goal_height + rules.arena.goal_side_radius
          ),
          rules.arena.goal_side_radius
        )
      );
    }
    // Top corner
    Vec2D o = {
      rules.arena.goal_width/2.0 - rules.arena.goal_top_radius,
      rules.arena.goal_height - rules.arena.goal_top_radius
    };
    v = Vec2D(point.x, point.y) - o;
    if (v.x > 0 and v.z > 0) {
      o += v.normalize() * (rules.arena.goal_top_radius + rules.arena.goal_side_radius);
      dan = std::min(
        dan,
        dan_to_sphere_outer(
          point,
          Vec3D(
            o.x,
            rules.arena.depth/2.0 + rules.arena.goal_side_radius,
            o.z
          ),
          rules.arena.goal_side_radius
        )
      );
    }
  }

  // Goal inside top corners
  if (point.z > rules.arena.depth/2.0 + rules.arena.goal_side_radius and
      point.y > rules.arena.goal_height - rules.arena.goal_top_radius) {
    // Side x
    if (point.x > rules.arena.goal_width/2.0 - rules.arena.goal_top_radius) {
      dan = std::min(
        dan,
        dan_to_sphere_inner(
          point,
          Vec3D(
            rules.arena.goal_width/2 - rules.arena.goal_top_radius,
            point.z,
            rules.arena.goal_height - rules.arena.goal_top_radius
          ),
          rules.arena.goal_top_radius
        )
      );
    }
    // Side z
    if (point.z > rules.arena.depth/2 + rules.arena.goal_depth - rules.arena.goal_top_radius) {
      dan = std::min(
        dan,
        dan_to_sphere_inner(
          point,
          Vec3D(
            point.x,
            rules.arena.depth/2 + rules.arena.goal_depth - rules.arena.goal_top_radius,
            rules.arena.goal_height - rules.arena.goal_top_radius
          ),
          rules.arena.goal_top_radius
        )
      );
    }
  }

  // Bottom corners
  if (point.y < rules.arena.bottom_radius) {
    // Side x
    if (point.x > rules.arena.width/2 - rules.arena.bottom_radius) {
      dan = std::min(
        dan,
        dan_to_sphere_inner(
          point,
          Vec3D(
            rules.arena.width/2 - rules.arena.bottom_radius,
            point.z,
            rules.arena.bottom_radius
          ),
          rules.arena.bottom_radius
        )
      );
    }
    // Side z
    if (point.z > rules.arena.depth/2 - rules.arena.bottom_radius and
        point.x >= rules.arena.goal_width/2 + rules.arena.goal_side_radius) {
      dan = std::min(
        dan,
        dan_to_sphere_inner(
          point,
          Vec3D(
            point.x,
            rules.arena.depth/2 - rules.arena.bottom_radius,
            rules.arena.bottom_radius
          ),
          rules.arena.bottom_radius
        )
      );
    }
    // Side z (goal)
    if (point.z > rules.arena.depth/2 + rules.arena.goal_depth - rules.arena.bottom_radius) {
      dan = std::min(
        dan,
        dan_to_sphere_inner(
          point,
          Vec3D(
            point.x,
            rules.arena.depth/2 + rules.arena.goal_depth - rules.arena.bottom_radius,
            rules.arena.bottom_radius
          ),
          rules.arena.bottom_radius
        )
      );
    }
    // Goal outer corner
    Vec2D o = {
      rules.arena.goal_width/2 + rules.arena.goal_side_radius,
      rules.arena.depth/2 + rules.arena.goal_side_radius
    };
    Vec2D v = Vec2D(point.x, point.z) - o;
    if (v.x < 0 and v.z < 0 and v.len() < rules.arena.goal_side_radius + rules.arena.bottom_radius) {
      o += v.normalize() * (rules.arena.goal_side_radius + rules.arena.bottom_radius);
      dan = std::min(
        dan,
        dan_to_sphere_inner(
          point,
          Vec3D(
            o.x,
            o.z,
            rules.arena.bottom_radius
          ),
          rules.arena.bottom_radius
        )
      );
    }
    // Side x (goal)
    if (point.z >= rules.arena.depth/2 + rules.arena.goal_side_radius and
        point.x > rules.arena.goal_width/2 - rules.arena.bottom_radius) {
      dan = std::min(
        dan,
        dan_to_sphere_inner(
          point,
          Vec3D(
            rules.arena.goal_width/2 - rules.arena.bottom_radius,
            point.z,
            rules.arena.bottom_radius
          ),
          rules.arena.bottom_radius
        )
      );
    }
    // corner
    if (point.x > rules.arena.width/2 - rules.arena.corner_radius and
        point.z > rules.arena.depth/2 - rules.arena.corner_radius) {
      Vec2D corner_o = {
        rules.arena.width/2 - rules.arena.corner_radius,
        rules.arena.depth/2 - rules.arena.corner_radius
      };
      Vec2D n = Vec2D(point.x, point.z) - corner_o;
      double dist = n.len();
      if (dist > rules.arena.corner_radius - rules.arena.bottom_radius) {
        n *= 1/dist;
        Vec2D o2 = corner_o + n * (rules.arena.corner_radius - rules.arena.bottom_radius);
        dan = std::min(
          dan,
          dan_to_sphere_inner(
            point,
            Vec3D(
              o2.x,
              o2.z,
              rules.arena.bottom_radius
            ),
            rules.arena.bottom_radius
          )
        );
      }
    }
  }

  // Ceiling corners
  if (point.y > rules.arena.height - rules.arena.top_radius) {
    // Side x
    if (point.x > rules.arena.width/2 - rules.arena.top_radius) {
      dan = std::min(
        dan,
        dan_to_sphere_inner(
          point,
          Vec3D(
            rules.arena.width/2 - rules.arena.top_radius,
            point.z,
            rules.arena.height - rules.arena.top_radius
          ),
          rules.arena.top_radius
        )
      );
    }
    // Side z
    if (point.z > rules.arena.depth/2 - rules.arena.top_radius) {
      dan = std::min(
        dan,
        dan_to_sphere_inner(
          point,
          Vec3D(
            point.x,
            rules.arena.depth/2 - rules.arena.top_radius,
            rules.arena.height - rules.arena.top_radius
          ),
          rules.arena.top_radius
        )
      );
    }
    // Side Corner
    if (point.x > rules.arena.width/2 - rules.arena.corner_radius and
        point.z > rules.arena.depth/2 - rules.arena.corner_radius) {
      Vec2D corner_o = {
        rules.arena.width/2 - rules.arena.corner_radius,
        rules.arena.depth/2 - rules.arena.corner_radius
      };
      Vec2D dv = Vec2D(point.x, point.z) - corner_o;
      if (dv.len() > rules.arena.corner_radius - rules.arena.top_radius) {
        Vec2D n = dv.normalize();
        Vec2D o2 = corner_o + n * (rules.arena.corner_radius - rules.arena.top_radius);
        dan = std::min(
          dan,
          dan_to_sphere_inner(
              point,
              Vec3D (
                o2.x,
                o2.z,
                rules.arena.height - rules.arena.top_radius
              ),
              rules.arena.top_radius
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
