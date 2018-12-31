#ifndef _SIM_PREDICT_CPP_
#define _SIM_PREDICT_CPP_

#include "Simulation.h"
#include "PointVectors.h"

#include <iostream>

Path Simulation::get_jump_path(const Entity &en) {
  Entity enc = en;

  assert(enc.type == ALLY or enc.type == ENEMY);

  Path jump_path = {enc.position};
  //std::cout<<enc.velocity.str()<<"|"<<enc.position.str()<<"|"<<dan_to_arena(enc.position).distance<<"|"<<enc.radius<<"\n";

  double distance_to_arena = dan_to_arena(enc.position).distance;

  int i = 0;
  do {
    if (i == 0 and enc.radius >= distance_to_arena)
      jump(enc, rules.ROBOT_MAX_JUMP_SPEED, i);
    if (enc.last_sim_jump != i)
      unjump(enc);
    collide_with_arena(enc);
    jump_path.push_back(enc.position);
    
    move(enc);

    //std::cout<<enc.velocity.str()<<"|"<<enc.position.str()<<"|"<<distance_to_arena<<"|"<<enc.radius<<"\n";

    distance_to_arena = dan_to_arena(enc.position).distance;
    ++i;
  } while (distance_to_arena > rules.ROBOT_RADIUS);
  //std::cout<<"---------------------\n";

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
    move(enc);
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
