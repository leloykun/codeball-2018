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
#include <string>
#include <vector>
#include <tuple>
#include <map>

// #include <iostream>

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
  double GOAL_EDGE;
  double REACHABLE_HEIGHT;
  // double ACCEPTABLE_JUMP_DISTANCE;

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
  Target t_attack_aggro;
  Target t_cross;
  Target t_block;
  Target t_follow;

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
  void calc_targets();
  Role calc_role();
  void set_action(
      model::Action &action,
      const int &id,
      const Vec3D &target_position,
      const Vec3D &target_velocity,
      const double &jump_speed,
      const bool &use_nitro);

  Target calc_intercept_spot(
      const double &reachable_height,
      const bool &to_shift_x,
      const double &z_offset,
      const double &min_speed,
      const double &max_speed);
  Target calc_defend_spot();
  Target calc_block_spot(const double &offset);
  Target calc_follow_spot(const double &z_offset);

  bool is_duplicate_target(
      const Vec2D &position,
      const double &acceptable_delta);
  std::tuple<bool, Vec3D, double> get_first_reachable_by(const int &id);
  int get_id_pos_enemy_attacker(const Vec2D &position);
  bool can_arrive_earlier_than_enemies(const Vec2D &position);

  double calc_jump_speed(const double &acceptable_jump_dist);
  std::tuple<bool, Vec3D, Vec3D> calc_valid_jump_intercept(
      const Path &robot_path,
      const Path &ball_path,
      const Vec3D &robot_position);
  /*
  bool is_closest_to_our_goal();
  bool is_closer_than_enemies(const Vec2D &pos);
  bool can_enemy_interrupt_before_us(const double &time_diff);
  std::tuple<Vec2D&, std::vector<Vec2D>& > calc_targets_from(
      const PosVelTime &robot_pvt,
      const PosVelTime &ball_pvt,
      const int &num_rays);
  */
  std::string custom_rendering() override;
};

#endif
