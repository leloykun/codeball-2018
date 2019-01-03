#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif

#ifndef _SIM_PREDICT_CPP_
#define _SIM_PREDICT_CPP_

#include "Simulation.h"
#include "PointVectors.h"

#include <iostream>

Path Simulation::get_jump_path(
    const Entity &en,
    const double &delta_time,
    const double &jump_speed,
    const int &jump_on_tick) {
  Entity enc = en;

  assert(enc.type == ALLY or enc.type == ENEMY);

  Path jump_path = {enc.position};
  //std::cout<<enc.velocity.str()<<"|"<<enc.position.str()<<"|"<<enc.radius<<"\n";

  double t = 0.0;
  int num_collisions_with_arena = 0;

  for (int tick = 0; tick < 60; ++tick) {
    for (double partition_size : TICK_PARTITION) {
      move(enc, delta_time * partition_size);
      if (tick == jump_on_tick)
        jump(enc, jump_speed, tick);
      else
        unjump(enc);
      enc.radius = rules.ROBOT_MAX_RADIUS;
      enc.radius_change_speed = rules.ROBOT_MAX_JUMP_SPEED;
      if (collide_with_arena(enc))
        num_collisions_with_arena++;
      //std::cout<<enc.velocity.str()<<"|"<<enc.position.str()<<"|"<<enc.radius<<"\n";
      t += delta_time * partition_size;
    }

    enc.position.t = t;
    jump_path.push_back(enc.position);

    if (num_collisions_with_arena == 2)
      break;
  }

  return jump_path;
}

JumpBallIntercept Simulation::simulate_jump(
    const Entity &en,
    const Entity &ball,
    const double &delta_time,
    const double &jump_speed,
    const int &jump_on_tick) {
  Entity enc = en;          assert(enc.type == ALLY or enc.type == ENEMY);
  Entity ballc = ball;      assert(ballc.type == BALL);

  Path robot_path = {enc.position};
  Path ball_path = {ballc.position};

  double t = 0.0;
  bool ball_is_intercepted = false;
  Vec3D robot_intercept_position;
  Vec3D ball_intercept_position;
  bool can_score = false;

  int num_collisions_with_arena = 0;

  int ticks_left = 4*60;

  for (int tick = 0; tick < 4*60; ++tick) {
    for (double partition_size : TICK_PARTITION) {
      move(enc, delta_time * partition_size);
      if (tick == jump_on_tick)
        jump(enc, jump_speed, tick);
      else
        unjump(enc);
      move(ballc, delta_time * partition_size);

      if (collide_entities(enc, ballc)) {
        ball_is_intercepted = true;
        robot_intercept_position = enc.position;
        ball_intercept_position = ballc.position;
      }
      if (collide_with_arena(enc))
        num_collisions_with_arena++;
      collide_with_arena(ballc);

      t += delta_time * partition_size;
    }

    enc.position.t = t;
    ballc.position.t = t;
    robot_path.push_back(enc.position);
    ball_path.push_back(ballc.position);

    ticks_left--;
    if (goal_scored(ballc.position.z)) {
      if (ballc.position.z >= arena.depth/2.0 + rules.BALL_RADIUS)
        can_score = true;
      ticks_left = 0;
      break;
    }
    if (ball_is_intercepted or num_collisions_with_arena >= 2)
      break;
  }

  for (int tick = 0; tick < ticks_left; ++tick) {
    for (double partition_size : TICK_PARTITION) {
      move(ballc, delta_time * partition_size);
      collide_with_arena(ballc);

      t += delta_time * partition_size;
    }

    ballc.position.t = t;
    ball_path.push_back(ballc.position);

    if (goal_scored(ballc.position.z)) {
      if (ballc.position.z >= arena.depth/2.0 + rules.BALL_RADIUS)
        can_score = true;
      break;
    }
  }

  return {
      ball_is_intercepted,
      can_score,
      robot_path,
      ball_path,
      robot_intercept_position,
      ball_intercept_position};
}

/*
Path Simulation::get_defence_path(
    const Entity &en,
    const int &till_tick,
    const double &jump_speed) {
  Entity enc = en;

  assert(enc.type == ALLY or enc.type == ENEMY);

  Path jump_path = {enc.position};

  for (int i = 1; i < till_tick; ++i) {
    move(enc, 1/100.0);
    if (is_touching_arena(enc) and enc.velocity.x <= -(rules.ROBOT_MAX_GROUND_SPEED-1))
      jump(enc, jump_speed, i);
    if (enc.last_sim_jump != i)
      unjump(enc);
    collide_with_arena(enc);
    jump_path.push_back(enc.position);
    //std::cout<<enc.velocity.str()<<"|"<<enc.position.str()<<"\n";
  }

  //std::cout<<"---------------------\n";

  return jump_path;
}
*/

#endif
