#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif

#ifndef _ENTITY_H_
#define _ENTITY_H_

#include "model/Rules.h"
#include "model/Game.h"
#include "model/Action.h"
#include "model/Robot.h"

#include "PointVectors.h"

/*
enum Role {
  GOALKEEPER,
  ATTACKER,
  BLOCKER
};
*/

enum ActionSeq {
  GOALKEEPING = 0,
  CLEARING_BALL = 1,
  PREPARING_TO_ATTACK = 2,
  ATTACKING = 3,
  BLOCKING = -1
};

enum EntityType {ENEMY, BALL, ALLY, PVT};

struct EntityLite {
  int id;
  EntityType type;
  Vec3D position;
  Vec3D velocity;
  double radius;
  double radius_change_speed;
  double mass;
  double coeff_restitution_arena;

  Vec3D target_velocity;
  int last_jump_tick = -1;
};

struct Entity {
  int id;
  EntityType type;

  Vec3D position;
  Vec3D velocity;

  double radius;
  double radius_change_speed = 0.0;
  double mass;
  double arena_e;

  Path projected_path;

  // for balls:
  std::vector<Vec3D> bounce_positions;

  // for robots:
  Vec3D target_position;
  Vec3D target_velocity;
  double jump_speed;
  int last_jump_tick = -1;
  bool use_nitro;
  // Role role;
  ActionSeq action_seq;
  Path projected_jump_path;

  Entity() { }
  Entity(const model::Ball &ball, const model::Rules RULES);
  Entity(const model::Robot &robot, const model::Rules RULES);

  void update(const model::Ball &ball);
  void update(const model::Robot &robot);

  EntityLite lighten() const;
  PosVelTime strip() const;
};

#endif
