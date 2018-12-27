#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif

#ifndef _SIMULATION_H_
#define _SIMULATION_H_

#include "model/Rules.h"
#include "model/Game.h"
#include "model/Action.h"
#include "model/Robot.h"

#include "PointVectors.h"

enum EntityType {ENEMY, BALL, ALLY};

struct Entity {
  Vec3D position;
  Vec3D velocity;
  Vec3D target_velocity;
  double radius;
  double radius_change_speed;
  double mass;
  double coeff_restitution_arena;
  // -1 for enemies
  //  0 for the ball
  //  1 for allies
  EntityType type;
  int id = 0;
  int last_sim_jump = -1;
};

struct DaN {
  double distance;
  Vec3D normal;
  bool operator<(const DaN &other) const { return distance < other.distance; }
};

class Simulation {
  int sim_tick = 0;
  double delta_time;
  model::Rules rules;
  model::Arena arena;
public:
  Entity ball;
  Entity ball_spec;
  std::vector<Entity> robots;

  Simulation(const model::Ball &ball,
             const std::vector<model::Robot> &robots,
             const std::vector<Vec3D> &target_velocities,
             const std::vector<double> &jump_speeds,
             const model::Rules &rules,
             double delta_time);
  double clamp(double a, double min_val, double max_val);
  Vec3D clamp(const Vec3D &v, double val);
  void update();
  void move(Entity &en);
  void move(std::vector<Entity> &robots);
  bool might_jump(Entity &en);
  void jump(Entity &en);
  void unjump(Entity &en);
  bool collide_entities(Entity &a, Entity &b);
  DaN collide_with_arena(Entity &en);
  DaN dan_to_plane(const Vec3D &point, const Vec3D &point_on_plane, const Vec3D &plane_normal);
  DaN dan_to_sphere_inner(const Vec3D &point, const Vec3D &sphere_center, double sphere_radius);
  DaN dan_to_sphere_outer(const Vec3D &point, const Vec3D &sphere_center, double sphere_radius);
  DaN dan_to_arena_quarter(const Vec3D &point);
  DaN dan_to_arena(Vec3D &point);
};

#endif
