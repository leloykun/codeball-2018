#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif

#ifndef _ENTITY_CPP_
#define _ENTITY_CPP_

#include "Entity.h"

Entity::Entity(const model::Ball &ball, const model::Rules RULES) {
  this->id = -1;
  this->type = BALL;
  this->position = {ball.x, ball.z, ball.y};
  this->velocity = {ball.velocity_x, ball.velocity_z, ball.velocity_y};

  this->radius = RULES.BALL_RADIUS;
  this->mass = RULES.BALL_MASS;
  this->arena_e = RULES.BALL_ARENA_E;
}

Entity::Entity(const model::Robot &robot, const model::Rules RULES) {
  this->id = robot.id;
  this->type = robot.is_teammate ? ALLY : ENEMY;
  this->position = {robot.x, robot.z, robot.y};
  this->velocity = {robot.velocity_x, robot.velocity_z, robot.velocity_y};

  this->radius = RULES.ROBOT_RADIUS;
  this->mass = RULES.ROBOT_MASS;
  this->arena_e = RULES.ROBOT_ARENA_E;
}

void Entity::update(const model::Ball &ball) {
  this->position = {ball.x, ball.z, ball.y};
  this->velocity = {ball.velocity_x, ball.velocity_z, ball.velocity_y};
}

void Entity::update(const model::Robot &robot) {
  this->position = {robot.x, robot.z, robot.y};
  this->velocity = {robot.velocity_x, robot.velocity_z, robot.velocity_y};
  this->radius = robot.radius;
}

EntityLite Entity::lighten() const {
  return {
    this->id,
    this->type,
    this->position,
    this->velocity,
    this->radius,
    this->radius_change_speed,
    this->mass,
    this->arena_e,
    this->target_velocity,
    this->last_jump_tick
  };
}

PosVelTime Entity::strip() const {
  return {
    this->position,
    this->velocity,
    0.0
  };
}

#endif
