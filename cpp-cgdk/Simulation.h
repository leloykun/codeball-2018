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
#include "GeomUtils.h"
#include "Entity.h"

#include <iostream>
#include <random>
#include <vector>
#include <cassert>
#include <map>
#include <tuple>
#include <memory>

const std::vector<double> TICK_PARTITION = {0.01, 0.01, 0.18, 0.20, 0.20, 0.20, 0.20};
// const std::vector<double> TICK_PARTITION = {0.01, 0.01, 0.98};

enum JumpMethod {BY_TICK, BY_SPEED, DONT_JUMP};

struct DaN {
  double distance;
  Vec3D normal;
  bool operator<(const DaN &other) const {
    return distance < other.distance;
  }
};

struct Simulation {
  model::Rules rules;
  model::Arena arena;

  Simulation () { }
  Simulation(const model::Rules &rules);
  void move(EntityLite &en, const double &delta_time);
  bool collide_entities(EntityLite &a, EntityLite &b);
  bool collide_with_arena(EntityLite &en);
  void jump(EntityLite &en, const double &jump_speed, const int &on_tick);
  void unjump(EntityLite &en);
  bool is_touching_arena(EntityLite &en);
  bool goal_scored(const double &z);

  // ----------- Found in: SimPredict.cpp ---------------
  void calc_ball_path(
      Entity &ball,
      const int &num_ticks,
      const double &delta_time);
  void calc_ball_path(
      Path &projected_path,
      std::vector<Vec3D> &bounce_positions,
      const EntityLite &ball,
      const int &num_ticks,
      const double &delta_time);
  void calc_robot_path(
      Entity &robot,
      Path &robot_path,
      const double &delta_time,
      const double &jump_speed,
      const JumpMethod &jump_method = BY_TICK,
      const int &jump_on_tick = 0,
      const double &jump_on_speed = 0.0);
  void calc_robot_path(
      Path &projected_path,
      const EntityLite &robot,
      const double &delta_time,
      const double &jump_speed,
      const JumpMethod &jump_method = BY_TICK,
      const int &jump_on_tick = 0,
      const double &jump_on_speed = 0.0);
  double calc_travel_time(
      const Entity &robot,
      const Vec2D &target_position);
  std::tuple<bool, Vec3D, double> calc_ball_intercept(
      const Entity &robot,
      const Entity &ball,
      const double &reachable_height,
      const double &time_lim=INF);
  std::unique_ptr<Path> calc_travel_path(
      const Entity &robot,
      const Vec2D &target);

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
