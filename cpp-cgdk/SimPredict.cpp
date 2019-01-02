#ifndef _SIM_PREDICT_CPP_
#define _SIM_PREDICT_CPP_

#include "Simulation.h"
#include "PointVectors.h"

#include <iostream>

Path Simulation::get_jump_path(const Entity &en, const double &delta_time) {
  Entity enc = en;

  assert(enc.type == ALLY or enc.type == ENEMY);

  Path jump_path = {enc.position};
  //std::cout<<enc.velocity.str()<<"|"<<enc.position.str()<<"|"<<enc.radius<<"\n";

  int num_collisions_with_arena = 0;

  for (int tick = 0; tick < 60; ++tick) {
    for (double partition_size : TICK_PARTITION) {
      move(enc, delta_time * partition_size);
      enc.radius = rules.ROBOT_MAX_RADIUS;
      enc.radius_change_speed = rules.ROBOT_MAX_JUMP_SPEED;
      if (collide_with_arena(enc))
        num_collisions_with_arena++;
      jump_path.push_back(enc.position);
      //std::cout<<enc.velocity.str()<<"|"<<enc.position.str()<<"|"<<enc.radius<<"\n";
    }
    if (num_collisions_with_arena == 2)
      break;
  }

  return jump_path;
}

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

#endif
