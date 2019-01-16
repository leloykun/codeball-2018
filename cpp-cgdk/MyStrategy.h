#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif

#ifndef _MY_STRATEGY_H_
#define _MY_STRATEGY_H_

#include "Strategy.h"
#include "PointVectors.h"
#include "Simulation.h"
#include "RenderUtil.h"
#include "GeomUtils.h"
#include "Entity.h"
#include <iostream>
#include <string>
#include <vector>
#include <tuple>
#include <map>

const int SIMULATION_NUM_TICKS = 240;
const double SIMULATION_PRECISION = 1/60.0;
const int NUM_RAYS = 20;

struct Target {
  bool exists;
  Vec2D position;
  Vec2D needed_velocity;
};

class MyStrategy : public Strategy {
  model::Rules RULES;
  model::Arena ARENA;

  double ZONE_BORDER;
  double DEFENSE_BORDER;
  double CRITICAL_BORDER;
  double REACHABLE_HEIGHT;
  double ACCEPTABLE_JUMP_DISTANCE;

  Vec2D GOAL_LIM_LEFT;
  Vec2D GOAL_LIM_RIGHT;

  Simulation sim;
  RenderUtil renderer;

  // initialized after init_strategy()
  Entity ball;
  std::map<int, Entity> robots;
public:
  MyStrategy();

  bool initialized = false;
  int current_tick = -1;

  std::vector<int> robot_ids;
  std::vector<int> ally_ids;
  std::vector<int> enemy_ids;

  int me_id;
  Entity *me;

  Target t_attack;
  Target t_defend;
  Target t_prepare;
  Target t_block;

  void act(
      const model::Robot &me,
      const model::Rules &rules,
      const model::Game &game,
      model::Action &action) override;

  void init_strategy(
      const model::Rules &rules,
      const model::Game &game);
  void init_tick(const model::Game &game);
  void init_query(const int &me_id, const model::Game &game);
  void run_simulation(const model::Game &game);
  ActionSeq calc_action(model::Action &action, const int &num_rays);
  void set_action(
      model::Action &action,
      const ActionSeq &action_seq,
      const Vec3D &target_position,
      const Vec3D &target_velocity,
      const double &jump_speed,
      const bool &use_nitro);

  bool is_closest_to_our_goal();
  bool is_closer_than_enemies(const Vec2D &pos);
  bool can_enemy_interrupt_before_us(const double &time_diff);
  bool an_ally_is_attacking();
  bool duplicate_action(const ActionSeq &action_seq);
  bool duplicate_target(
      const Vec2D &target_position,
      const double &acceptable_dist);
  Target calc_attack(const int &num_rays, const double &min_speed);
  Target calc_defend();
  Target calc_prepare(const double &pos_delta);
  Target calc_block(const double &pos_delta);
  std::tuple<Vec2D&, std::vector<Vec2D>& > calc_targets_from(
      const PosVelTime &robot_pvt,
      const PosVelTime &ball_pvt,
      const int &num_rays);
  double calc_jump_speed();
  std::tuple<bool, Vec3D, Vec3D> calc_valid_jump_intercept(
      const Path &robot_path,
      const Path &ball_path,
      const Vec3D &robot_position);

  std::string custom_rendering() override;
};

#endif
