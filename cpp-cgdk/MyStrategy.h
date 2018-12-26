#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif

#ifndef _MY_STRATEGY_H_
#define _MY_STRATEGY_H_

#include "Strategy.h"
#include "PointVectors.h"
#include "Simulation.h"
#include <string>
#include <vector>
#include <iostream>

const double EPS = 1e-5;

// enum Role {GUARD, ATTACKER, AGGRESSIVE_GUARD, SPECULATIVE_ATTACKER};

class MyStrategy : public Strategy {
public:
  MyStrategy();

  model::Rules rules;

  int prev_tick = -1;
  std::vector<Vec3D> predicted_ball_positions;
  std::vector<Vec3D> predicted_ball_col_positions;
  std::vector<std::vector<Vec3D> > predicted_robot_positions;

  std::vector<int> ally_ids;

  std::vector<Vec3D> target_positions = {Vec3D()};
  std::vector<Vec3D> target_velocities = {Vec3D()};
  std::vector<double> jump_speeds = {0.0};
  // std::vector<Role> roles = {GUARD};

  void act(const model::Robot& me, const model::Rules& rules, const model::Game& game, model::Action& action) override;
  bool is_duplicate_target(const Vec2D &target_pos, const Vec2D &delta_pos, int id, const std::vector<model::Robot> &robots);
  void set_action(model::Action &action, int id, const Vec3D &target_position, const Vec3D &target_velocity, double jump_speed, bool use_nitro);

  std::string custom_rendering() override;
  std::string draw_sphere_util(const Vec3D &pos, double radius, double r, double g, double b, double a);
  std::string draw_line_util(const Vec3D &p1, const Vec3D &p2, double width, double r, double g, double b, double a);
};

#endif
