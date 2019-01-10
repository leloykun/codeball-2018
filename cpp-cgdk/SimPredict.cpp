#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif

#ifndef _SIM_PREDICT_CPP_
#define _SIM_PREDICT_CPP_

#include "Simulation.h"

enum JumpState {BEFORE_JUMP, JUMPING, AFTER_JUMP};

void Simulation::calc_ball_path(
    Entity &ball,
    const int &num_ticks,
    const double &delta_time) {
  this->calc_ball_path(
    ball.projected_path,
    ball.bounce_positions,
    ball.lighten(),
    num_ticks,
    delta_time
  );
}

void Simulation::calc_ball_path(
    Path &projected_path,
    std::vector<Vec3D> &bounce_positions,
    const EntityLite &ball,
    const int &num_ticks,
    const double &delta_time) {
  EntityLite c_ball = ball;
  assert(c_ball.type == BALL);

  double t = 0.0;

  projected_path = {{c_ball.position, c_ball.velocity, t}};
  bounce_positions = {};


  for (int tick = 1; tick <= num_ticks; ++tick) {
    bool has_collided_with_arena = false;
    for (double partition_size : TICK_PARTITION) {
      this->move(c_ball, delta_time * partition_size);
      if (this->collide_with_arena(c_ball))
        has_collided_with_arena = true;
      t += delta_time * partition_size;
    }

    projected_path.push_back({c_ball.position, c_ball.velocity, t});

    if (has_collided_with_arena)
      bounce_positions.push_back(c_ball.position);

    if (this->goal_scored(c_ball.position.z))
      return;
  }
}

void Simulation::calc_robot_path(
    Entity &robot,
    Path &robot_path,
    const double &delta_time,
    const double &jump_speed,
    const JumpMethod &jump_method,
    const int &jump_on_tick,
    const double &jump_on_speed) {
  this->calc_robot_path(
    robot_path,
    robot.lighten(),
    delta_time,
    jump_speed,
    jump_method,
    jump_on_tick,
    jump_on_speed
  );
}

void Simulation::calc_robot_path(
    Path &projected_path,
    const EntityLite &robot,
    const double &delta_time,
    const double &jump_speed,
    const JumpMethod &jump_method,
    const int &jump_on_tick,
    const double &jump_on_speed) {
  EntityLite c_robot = robot;
  assert(c_robot.type == ALLY or c_robot.type == ENEMY);

  double t = 0.0;

  projected_path = {{c_robot.position, c_robot.velocity, t}};

  JumpState state = (this->is_touching_arena(c_robot) ? BEFORE_JUMP : JUMPING);

  for (int tick = 0; tick < 60; ++tick) {
    bool has_collided_with_arena = false;
    for (double partition_size : TICK_PARTITION) {
      this->move(c_robot, delta_time * partition_size);
      if (state == BEFORE_JUMP) {
        if ((jump_method == BY_TICK and tick == jump_on_tick) or
            (jump_method == BY_SPEED and std::fabs(c_robot.velocity.len() - jump_on_speed) < BIG_EPS))
          this->jump(c_robot, jump_speed, tick);
      } else
        this->unjump(c_robot);

      if (this->collide_with_arena(c_robot))
        has_collided_with_arena = true;
      t += delta_time * partition_size;
    }

    projected_path.push_back({c_robot.position, c_robot.velocity, t});

    if (has_collided_with_arena) {
      if (state == JUMPING)
        state = AFTER_JUMP;
    } else if (state != AFTER_JUMP)
      state = JUMPING;

    if (state == AFTER_JUMP)
      break;
  }
}

#endif
