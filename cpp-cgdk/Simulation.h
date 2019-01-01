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

struct DaN {
  double distance;
  Vec3D normal;
  bool operator<(const DaN &other) const {
    return distance < other.distance;
  }
};

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
  int id;
  int last_sim_jump = -1;
};

struct Simulation {
  model::Rules rules;
  model::Arena arena;
  int num_robots;

  int sim_tick;

  Entity ball;
  Entity ball_spec;
  std::vector<Entity> robots;

  Path proj_ball_path;
  Path proj_ball_spec_path;
  std::vector<Path> proj_robot_paths;

  Simulation () { }
  Simulation(
      const model::Ball &ball,
      const std::vector<model::Robot> &robots,
      const model::Rules &rules);
  void set(
      const model::Ball &ball,
      const std::vector<model::Robot> &robots,
      const std::vector<Vec3D> &target_velocities,
      const std::vector<double> &jump_speeds,
      const int &sim_tick);

  void run(
      const int &num_ticks,
      const double &delta_time);
  void update(const double &delta_time);
  void move(Entity &en, const double &delta_time);

  bool might_jump(Entity &en);
  void jump(Entity &en, const double &jump_speed, const int &tick);
  void unjump(Entity &en);

  bool is_touching_arena(Entity &en);
  void collide_entities(Entity &a, Entity &b);
  bool collide_with_arena(Entity &en);

  // ----------- Found in: SimPredict.cpp ---------------
  Path get_jump_path(const Entity &en);
  Path get_defence_path(
      const Entity &en,
      const int &till_tick,
      const double &jump_speed);

  // ----------- Found in: SimUtils.cpp ---------------
  DaN dan_to_plane(
      const Vec3D &point,
      const Vec3D &point_on_plane,
      const Vec3D &plane_normal);
  DaN dan_to_sphere_inner(
      const Vec3D &point,
      const Vec3D &sphere_center,
      const double &sphere_radius);
  DaN dan_to_sphere_outer(
      const Vec3D &point,
      const Vec3D &sphere_center,
      const double &sphere_radius);
  DaN dan_to_arena_quarter(const Vec3D &point);
  DaN dan_to_arena(Vec3D &point);
};

#endif
