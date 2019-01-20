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

double Simulation::calc_travel_time(
    const Entity &robot,
    const Vec2D &target_position) {
  double time = 0.0;
  Vec2D from_position = robot.position.drop();
  if (not robot.touch) {
    time += robot.projected_jump_path.back().time;
    from_position = robot.projected_jump_path.back().position.drop();
  }

  time += geom::time_to_go_to(
    from_position,
    robot.velocity.drop(),
    target_position
  );

  return time;
}

std::tuple<bool, Vec3D, double> Simulation::calc_ball_intercept(
    const Entity &robot,
    const Entity &ball,
    const double &height_lim,
    const double &time_lim) {
  double init_time = 0.0;
  Vec2D from_position = robot.position.drop();
  if (not robot.touch) {
    int N = std::min(int(robot.projected_jump_path.size()), int(ball.projected_path.size()));
    for (int i = 0; i < N; ++i) {
      auto robot_pvt = robot.projected_jump_path[i];
      auto ball_pvt = ball.projected_path[i];
      assert(std::fabs(robot_pvt.time - ball_pvt.time) < BIG_EPS);
      if (ball_pvt.time > time_lim)
        break;
      if ((robot_pvt.position - ball_pvt.position).len() <=
          this->rules.BALL_RADIUS + this->rules.ROBOT_RADIUS)
        return {true, ball_pvt.position, robot_pvt.time};
    }

    init_time = robot.projected_jump_path.back().time;
    from_position = robot.projected_jump_path.back().position.drop();
  }

  for (auto &ball_pvt : ball.projected_path) {
    if (this->goal_scored(ball_pvt.position.z))
      break;
    if (ball_pvt.time > time_lim)
      break;
    if (ball_pvt.position.y > height_lim)
      continue;

    double needed_time = init_time + geom::time_to_go_to(
      from_position,
      robot.velocity.drop(),
      ball_pvt.position.drop(),
      true
    );

    if (needed_time <= ball_pvt.time)
      return {true, ball_pvt.position, needed_time};
  }

  return {false, Vec3D(), INF};
}


#endif
