#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif

#ifndef _SIMULATION_CPP_
#define _SIMULATION_CPP_

#include "Simulation.h"

using namespace model;

Simulation::Simulation(const Rules &rules) {
  this->rules = rules;
  this->arena = rules.arena;
}

void Simulation::move(EntityLite &en, const double &delta_time) {
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

bool Simulation::collide_entities(EntityLite &a, EntityLite &b) {
  const double AVE_HIT_E = (rules.MIN_HIT_E + rules.MAX_HIT_E) / 2.0;
  // TODO
  Vec3D delta_position = b.position - a.position;
  double distance = delta_position.len();
  double penetration = a.radius + b.radius - distance;
  if (penetration > 0) {
    double k_a = (1/a.mass) / ((1/a.mass) + (1/b.mass));
    double k_b = (1/b.mass) / ((1/a.mass) + (1/b.mass));
    Vec3D normal = delta_position.normalize();
    a.position -= normal * penetration * k_a;
    b.position += normal * penetration * k_b;
    double delta_velocity = (b.velocity - a.velocity).dot(normal) -
                            b.radius_change_speed - a.radius_change_speed;
    if (delta_velocity < 0) {
      Vec3D impulse = normal * (1 + AVE_HIT_E) * delta_velocity;
      a.velocity += impulse * k_a;
      b.velocity -= impulse * k_b;
    }
    return true;
  }
  return false;
}

bool Simulation::collide_with_arena(EntityLite &en) {
  DaN arena_collision = dan_to_arena(en.position);
  double penetration = en.radius - arena_collision.distance;
  if (penetration > 0) {
    en.position += arena_collision.normal * penetration;
    double velocity = en.velocity.dot(arena_collision.normal) - en.radius_change_speed;
    if (velocity < 0)
      en.velocity -= arena_collision.normal * (1 + en.coeff_restitution_arena) * velocity;
    return true;
  }
  return false;
}

void Simulation::jump(
    EntityLite &en,
    const double &jump_speed,
    const int &on_tick) {
  en.radius = rules.ROBOT_MIN_RADIUS +
              (rules.ROBOT_MAX_RADIUS - rules.ROBOT_MIN_RADIUS) *
                 (jump_speed / rules.ROBOT_MAX_JUMP_SPEED);
  en.radius_change_speed = jump_speed;
  en.last_jump_tick = on_tick;
}

void Simulation::unjump(EntityLite &en) {
  en.radius = rules.ROBOT_RADIUS;
  en.radius_change_speed = 0.0;
}

bool Simulation::is_touching_arena(EntityLite &en) {
  return en.radius >= dan_to_arena(en.position).distance;
}

bool Simulation::goal_scored(const double &z) {
  return std::fabs(z) > arena.depth/2.0 + rules.BALL_RADIUS;
}

#endif
