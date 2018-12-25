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

class MyStrategy : public Strategy {
public:
  MyStrategy();

  int prev_tick = -1;
  std::vector<Vec3D> predicted_ball_positions;
  std::vector<Vec3D> predicted_ball_col_positions;
  std::vector<std::vector<Vec3D> > predicted_robot_positions;

  std::vector<int> ally_ids;

  int cur_avail_id = 1;
  std::vector<Vec3D> target_positions = {Vec3D()};
  std::vector<Vec3D> target_velocities = {Vec3D()};
  std::vector<double> jump_speeds = {0.0};

  std::string draw_sphere_util(const Vec3D &pos, double radius, double r, double g, double b, double a);
  std::string draw_line_util(const Vec3D &p1, const Vec3D &p2, double width, double r, double g, double b, double a);
  std::string convert_positions_to_string();

  void act(const model::Robot& me, const model::Rules& rules, const model::Game& game, model::Action& action) override;
  std::string custom_rendering() override;
};

#endif
