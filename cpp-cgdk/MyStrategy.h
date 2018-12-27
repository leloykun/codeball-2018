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

struct Color {
  double r {0.0};
  double g {0.0};
  double b {0.0};
  Color() { }
  Color(double r, double g, double b) : r(r), g(g), b(b) { }
};

const Color RED    (1.0, 0.0, 0.0);
const Color GREEN  (0.0, 1.0, 0.0);
const Color BLUE   (0.0, 0.0, 1.0);
const Color VIOLET (1.0, 0.0, 1.0);
const Color WHITE  (1.0, 1.0, 1.0);
const Color BLACK  (0.0, 0.0, 0.0);
const Color LIGHT_RED  (1.0, 0.5, 0.5);
const Color LIGHT_BLUE (0.5, 0.5, 1.0);

enum Role {ATTACKER,              // red
           DEFENDER,              // blue
           AGGRESSIVE_DEFENDER,   // light-red
           SPECULATIVE_DEFENDER,  // light-blue
           SPECULATIVE_ATTACKER,  // violet
           DEFAULT};              // white

class MyStrategy : public Strategy {
  double DEFENSE_BORDER;
  double CRITICAL_BORDER;
  model::Rules rules;
public:
  MyStrategy();

  int prev_tick = -1;
  bool is_start_of_round = true;

  std::vector<Vec3D> predicted_ball_positions;
  std::vector<Vec3D> predicted_ball_col_positions;
  std::vector<std::vector<Vec3D> > predicted_robot_positions;

  std::vector<int> ally_ids;

  std::vector<Vec3D> target_positions = {Vec3D()};
  std::vector<Vec3D> target_velocities = {Vec3D()};
  std::vector<double> jump_speeds = {0.0};
  std::vector<Vec3D> robot_positions = {Vec3D()};
  std::vector<Role> roles = {DEFAULT};

  void act(const model::Robot& me,
           const model::Rules& rules,
           const model::Game& game,
           model::Action& action) override;
  bool find_intercept_spot(
           const std::vector<Vec3D> &predicted_ball_path,
           const Vec2D &my_position,
           Vec2D &target_position,
           Vec2D &target_velocity,
           const bool &is_speculative = false,
           const std::vector<Vec3D> &avoid_path = {});
  bool goal_scored(double z);
  bool is_duplicate_target(
           const Vec2D &target_position,
           const Vec2D &my_position,
           const int &id,
           const std::vector<model::Robot> &robots);
  void set_action(
           model::Action &action,
           const int &id,
           const Vec3D &target_position,
           const Vec3D &target_velocity,
           const double &jump_speed,
           const bool &use_nitro);

  std::string custom_rendering() override;
  std::string draw_border_util(const double &border_z);
  std::string draw_sphere_util(
           const Vec3D &pos,
           const double &radius,
           const Color &color,
           const double &alpha);
  std::string draw_line_util(
           const Vec3D &p1,
           const Vec3D &p2,
           const double &width,
           const Color &color,
           const double &alpha);
};

#endif
