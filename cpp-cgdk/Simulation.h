#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif

#ifndef _SIMULATION_H_
#define _SIMULATION_H_

#include "model/Rules.h"
#include "model/Game.h"
#include "model/Action.h"
#include "model/Robot.h"

#include "PointVectors.h"

struct Entity {
  Vec3D position;
  Vec3D velocity;
  double radius;
  double mass;
  double coeff_restitution_arena;
};

struct DaN {
  double distance;
  Vec3D normal;
  bool operator<(const DaN &other) const { return distance < other.distance; }
};

class Simulation {
  double delta_time;
  model::Rules rules;
public:
  Entity ball;
  std::vector<Entity> robots;

  Simulation(const model::Ball &ball,
             const std::vector<model::Robot> &robots,
             const model::Rules &rules,
             double delta_time);
  double clamp(double a, double min_val, double max_val);
  double clamp(double a, double max_val);
  void update();
  void move(Entity &en);
  void move(std::vector<Entity> &robots);
  void collide_entities(Entity &a, Entity &b);
  void collide_with_arena(Entity &en);
  DaN dan_to_plane(Vec3D point, Vec3D point_on_plane, Vec3D plane_normal);
  DaN dan_to_sphere_inner(Vec3D point, Vec3D sphere_center, double sphere_radius);
  DaN dan_to_sphere_outer(Vec3D point, Vec3D sphere_center, double sphere_radius);
  DaN dan_to_arena_quarter(const Vec3D &point);
  DaN dan_to_arena(Vec3D &point);
};

#endif
