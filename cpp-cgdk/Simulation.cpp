#ifndef _SIMULATION_CPP_
#define _SIMULATION_CPP_

#include "Simulation.h"
#include "PointVectors.h"
#include <iostream>
#include <random>
#include <vector>
#include <cassert>

using namespace model;

Simulation::Simulation(
    const Ball &ball,
    const std::vector<Robot> &robots,
    const Rules &rules) {
  this->rules = rules;
  this->arena = rules.arena;
  this->num_robots = robots.size();

  this->ball = {
    Vec3D(ball.x, ball.z, ball.y),
    Vec3D(ball.velocity_x, ball.velocity_z, ball.velocity_y),
    Vec3D(),
    rules.BALL_RADIUS,
    0.0,
    rules.BALL_MASS,
    rules.BALL_ARENA_E,
    BALL,
    -1
  };

  this->ball_spec = this->ball;

  this->robots = std::vector<Entity>(robots.size() + 1);
  for (Robot robot : robots) {
    this->robots[robot.id] = {
      Vec3D(robot.x, robot.z, robot.y),
      Vec3D(robot.velocity_x, robot.velocity_z, robot.velocity_y),
      Vec3D(),
      rules.ROBOT_RADIUS,
      0.0,
      rules.ROBOT_MASS,
      rules.ROBOT_ARENA_E,
      (robot.is_teammate ? ALLY : ENEMY),
      robot.id
    };
  }
}

void Simulation::set(
    const model::Ball &ball,
    const std::vector<model::Robot> &robots,
    const std::vector<Vec3D> &target_velocities,
    const std::vector<double> &jump_speeds,
    const int &sim_tick) {

  this->ball.position = {ball.x, ball.z, ball.y};
  this->ball.velocity = {ball.velocity_x, ball.velocity_z, ball.velocity_y};

  this->ball_spec.position = this->ball.position;
  this->ball_spec.velocity = this->ball.velocity;

  if (int(this->robots.size()) != int(robots.size()) + 1)
    this->robots = std::vector<Entity>(int(robots.size()) + 1);
  for (Robot robot : robots) {
    this->robots[robot.id].id = robot.id;
    this->robots[robot.id].position = {robot.x, robot.z, robot.y};
    this->robots[robot.id].velocity = {robot.velocity_x,
                                       robot.velocity_z,
                                       robot.velocity_y};
    this->robots[robot.id].target_velocity = target_velocities[robot.id];
    this->robots[robot.id].radius = robot.radius;
    this->robots[robot.id].radius_change_speed = jump_speeds[robot.id];
  }

  this->sim_tick = sim_tick;
}

void Simulation::run(
    const int &num_ticks,
    const double &delta_time) {
  // initiate projections
  proj_ball_path = {ball.position};
  proj_ball_spec_path = {ball_spec.position};

  proj_robot_paths = std::vector<Path>(robots.size());
  for (int id = 1; id <= num_robots; ++id)
    proj_robot_paths[id].push_back(robots[id].position);

  for (int tick = 1; tick <= num_ticks; ++tick) {
    update((1/100.0) * delta_time);
    update((99/100.0) * delta_time);
    this->sim_tick++;
  }
}

void Simulation::update(const double &delta_time) {
  for (int id = 1; id <= num_robots; ++id) {
    move(robots[id], delta_time);

    if (might_jump(robots[id]))
      jump(robots[id], rules.ROBOT_MAX_JUMP_SPEED, sim_tick);
    if (robots[id].last_sim_jump != this->sim_tick)
      unjump(robots[id]);
  }
  move(ball, delta_time);
  move(ball_spec, delta_time);

  for (int i = 1; i <= num_robots; ++i)
    for (int j = 1; j < i; ++j)
      collide_entities(robots[i], robots[j]);
  for (int id = 1; id <= num_robots; ++id) {
    // collide_entities(robots[id], ball);
    collide_entities(robots[id], ball_spec);

    collide_with_arena(robots[id]);
  }
  collide_with_arena(ball);
  collide_with_arena(ball_spec);

  proj_ball_path.push_back(ball.position);
  proj_ball_spec_path.push_back(ball_spec.position);
  for (int id = 1; id < int(robots.size()); ++id)
    proj_robot_paths[id].push_back(robots[id].position);
}

void Simulation::move(Entity &en, const double &delta_time) {
  if (en.type == ALLY or en.type == ENEMY) {
    DaN arena_collision = dan_to_arena(en.position);
    if (en.radius >= arena_collision.distance) {
      Vec3D target_velocity = clamp(en.target_velocity, rules.ROBOT_MAX_GROUND_SPEED);
      target_velocity -= arena_collision.normal * arena_collision.normal.dot(target_velocity);
      Vec3D target_velocity_change = target_velocity - en.velocity;
      //std::cout<<target_velocity_change.str()<<"|";
      if (target_velocity_change.len() > 0) {
        double acceleration = rules.ROBOT_ACCELERATION * std::max(0.0, arena_collision.normal.y);
        en.velocity += clamp(target_velocity_change.normalize() * acceleration * delta_time, target_velocity_change.len());
      }
    }
  }

  en.velocity.clamp(rules.MAX_ENTITY_SPEED);
  en.position += en.velocity * delta_time;
  en.position.y -= rules.GRAVITY * delta_time * delta_time / 2.0;
  en.velocity.y -= rules.GRAVITY * delta_time;
}

bool Simulation::might_jump(Entity &en) {
  if (!is_touching_arena(en))
    return false;
  // return true;
  double dist_to_ball = (en.position - ball_spec.position).len();
  if (en.type == ENEMY) {
    if (dist_to_ball < rules.BALL_RADIUS + 2*rules.ROBOT_MAX_RADIUS and
        en.position.z > ball_spec.position.z)
      return true;
  } else if (en.type == ALLY) {
    if (dist_to_ball < rules.BALL_RADIUS + 2*rules.ROBOT_MAX_RADIUS and
        en.position.z < ball_spec.position.z)
      return true;
  }
  return false;
}

//void Simulation::jump(Entity &en, const double &jump_speed, const int &tick) {
void Simulation::jump(Entity &en, const double &jump_speed, const int &tick) {
  en.radius = rules.ROBOT_MIN_RADIUS +
              (rules.ROBOT_MAX_RADIUS - rules.ROBOT_MIN_RADIUS) *
                 (jump_speed / rules.ROBOT_MAX_JUMP_SPEED);
  en.radius_change_speed = jump_speed;
  en.last_sim_jump = tick;
}

void Simulation::unjump(Entity &en) {
  en.radius = rules.ROBOT_MIN_RADIUS;
  en.radius_change_speed = 0.0;
}

bool Simulation::is_touching_arena(Entity &en) {
  return en.radius >= dan_to_arena(en.position).distance;
}

void Simulation::collide_entities(Entity &a, Entity &b) {
  const double AVE_HIT_E = (rules.MIN_HIT_E + rules.MAX_HIT_E) / 2.0;
  // TODO
  Vec3D delta_position = b.position - a.position;
  double distance = delta_position.len();
  double penetration = a.radius + b.radius - distance;
  if (penetration >= 0) {
    double k_a = (1/a.mass) / ((1/a.mass) + (1/b.mass));
    double k_b = (1/b.mass) / ((1/a.mass) + (1/b.mass));
    Vec3D normal = delta_position.normalize();
    a.position -= normal * penetration * k_a;
    b.position += normal * penetration * k_b;
    double delta_velocity = (b.velocity - a.velocity).dot(normal) -
                            b.radius_change_speed - a.radius_change_speed;
    if (delta_velocity <= 0) {
      Vec3D impulse = normal * (1 + AVE_HIT_E) * delta_velocity;
      a.velocity += impulse * k_a;
      b.velocity -= impulse * k_b;
    }
  }
}

bool Simulation::collide_with_arena(Entity &en) {
  DaN arena_collision = dan_to_arena(en.position);
  double penetration = en.radius - arena_collision.distance;
  if (penetration >= 0) {
    en.position += arena_collision.normal * penetration;
    double velocity = en.velocity.dot(arena_collision.normal) - en.radius_change_speed;
    if (velocity <= 0)
      en.velocity -= arena_collision.normal * (1 + en.coeff_restitution_arena) * velocity;
    return true;
  }
  return false;
}

#endif
