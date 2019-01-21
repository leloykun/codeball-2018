#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif

#ifndef _MY_STRATEGY_CPP_
#define _MY_STRATEGY_CPP_

#include "MyStrategy.h"

MyStrategy::MyStrategy() { }

void MyStrategy::act(
    const model::Robot &me,
    const model::Rules &rules,
    const model::Game &game,
    model::Action &action) {

  if (game.current_tick == 0 and not this->initialized)
    this->init_strategy(rules, game);

  if (this->current_tick != game.current_tick)
    this->init_tick(game);

  this->init_query(me.id, game);

  this->calc_targets();

  this->me->role = this->calc_role();

  Vec2D target_position;
  Vec2D target_velocity;

  switch (this->me->role) {
    case ATTACKER:
      target_position = this->t_attack.position;
      target_velocity = this->t_attack.needed_velocity;
      break;
    case AGGRESSIVE_DEFENDER:
      target_position = this->t_attack_aggro.position;
      target_velocity = this->t_attack_aggro.needed_velocity;
      break;
    case GOALKEEPER:
      target_position = this->t_cross.position;
      target_velocity = this->t_cross.needed_velocity;
      break;
    case BLOCKER:
      target_position = this->t_block.position;
      target_velocity = this->t_block.needed_velocity;
      break;
    case FOLLOWER:
      target_position = this->t_follow.position;
      target_velocity = this->t_follow.needed_velocity;
      break;
    default:
      assert(false);
      break;
  }

  this->set_action(
    action,
    this->me_id,
    Vec3D(target_position, 0.0),
    Vec3D(target_velocity, 0.0),
    this->calc_jump_speed(this->REACHABLE_HEIGHT),
    (not me.touch and this->me->velocity.y < 0)
  );
}


void MyStrategy::init_strategy(
    const model::Rules &rules,
    const model::Game &game) {
  if (VERBOSITY >= 1)
    std::cout << "START!\n";

  this->RULES = rules;
  this->ARENA = rules.arena;

  this->ZONE_BORDER = 0.0;
  this->DEFENSE_BORDER = -this->ARENA.depth/6.0;
  // this->CRITICAL_BORDER = -(this->ARENA.depth/2.0 - this->ARENA.top_radius);
  this->CRITICAL_BORDER = -this->ARENA.depth/2.0;
  this->GOAL_EDGE = this->ARENA.goal_width/2.0 - this->ARENA.goal_top_radius;
  this->REACHABLE_HEIGHT = this->RULES.ROBOT_MAX_RADIUS +
                          geom::calc_jump_height(
                            this->RULES.ROBOT_MAX_JUMP_SPEED,
                            this->RULES.GRAVITY) +
                          this->RULES.ROBOT_MIN_RADIUS +
                          this->RULES.BALL_RADIUS;
  // this->ACCEPTABLE_JUMP_DISTANCE = this->REACHABLE_HEIGHT;

  this->GOAL_LIM_LEFT  = Vec2D(  this->ARENA.goal_width/2.0 - 2*this->RULES.ROBOT_RADIUS,  this->ARENA.depth/2.0);
  this->GOAL_LIM_RIGHT = Vec2D(-(this->ARENA.goal_width/2.0 - 2*this->RULES.ROBOT_RADIUS), this->ARENA.depth/2.0);

  this->ball = Entity(game.ball, this->RULES);
  for (model::Robot robot : game.robots) {
    this->robots[robot.id] = Entity(robot, this->RULES);

    this->robot_ids.push_back(robot.id);
    (robot.is_teammate ? this->ally_ids : this->enemy_ids).push_back(robot.id);
  }

  this->sim = Simulation(rules);

  this->initialized = true;
}

void MyStrategy::init_tick(const model::Game &game) {
  assert(this->initialized);
  this->current_tick = game.current_tick;

  this->run_simulation(game);

  if (VERBOSITY >= 1 and this->current_tick % 100 == 0)
      std::cout << this->current_tick << "\n";
  else if (VERBOSITY >= 2) {
    std::cout << "----------------------\n";
    std::cout << "current tick: "<<this->current_tick<<"\n";
  }

  this->renderer.clear();
}

void MyStrategy::init_query(const int &me_id, const model::Game &game) {
  this->ball.update(game.ball);
  for (model::Robot robot : game.robots)
    this->robots[robot.id].update(robot);

  this->me_id = me_id;
  this->me = &this->robots[me_id];

  t_attack.exists = false;
  t_attack_aggro.exists = false;
  t_cross.exists = false;
  t_block.exists = false;
  t_follow.exists = false;
}

void MyStrategy::run_simulation(const model::Game &game) {
  this->sim.calc_ball_path(
    this->ball,
    SIMULATION_NUM_TICKS,
    SIMULATION_PRECISION);

  for (int id : this->robot_ids)
    this->sim.calc_robot_path(
      this->robots[id],
      this->robots[id].projected_jump_path,
      SIMULATION_PRECISION,
      RULES.ROBOT_MAX_JUMP_SPEED,
      BY_TICK,
      0);

  for (int id : this->robot_ids)
    this->sim.calc_robot_path(
      this->robots[id],
      this->robots[id].projected_path,
      SIMULATION_PRECISION,
      RULES.ROBOT_MAX_JUMP_SPEED,
      DONT_JUMP);

  for (int id : this->robot_ids) {
    this->robots[id].first_ball_intercept =
      this->sim.calc_ball_intercept(
        this->robots[id],
        this->ball,
        this->REACHABLE_HEIGHT
      );
  }
}

void MyStrategy::calc_targets() {
  t_attack = this->calc_intercept_spot(
    6*this->RULES.ROBOT_RADIUS,
    true,
    1.5*this->RULES.ROBOT_RADIUS,
    0.75*this->RULES.ROBOT_MAX_GROUND_SPEED,
    1.0*this->RULES.ROBOT_MAX_GROUND_SPEED);
  t_attack_aggro = this->calc_intercept_spot(
    6*this->RULES.ROBOT_RADIUS,
    false,
    1.5*this->RULES.ROBOT_RADIUS,
    0.5*this->RULES.ROBOT_MAX_GROUND_SPEED,
    1.0*this->RULES.ROBOT_MAX_GROUND_SPEED);
  t_cross = this->calc_defend_spot();
  t_block = this->calc_block_spot(2*this->RULES.BALL_RADIUS);
  t_follow = this->calc_follow_spot(2*this->RULES.BALL_RADIUS);
}

Role MyStrategy::calc_role() {
  Role role = (this->robots.size() == 2 ? ATTACKER : GOALKEEPER);

  for (int id : this->ally_ids)
    if (id != this->me_id and
        this->robots[id].position.z < this->me->position.z)
      role = ATTACKER;

  auto [en_intercept_exists, intercept_ball_pos, en_id] =
    this->calc_enemy_first_intercept(this->t_attack_aggro.needed_time);

  if (role == GOALKEEPER and
      this->t_attack_aggro.exists and
      this->t_attack_aggro.position.z <= this->DEFENSE_BORDER and
      not en_intercept_exists) {
    role = AGGRESSIVE_DEFENDER;
  }

  if (role == ATTACKER and
      this->t_attack.exists and
      not this->is_duplicate_target(this->t_attack.position,
                                    this->RULES.BALL_RADIUS) and
      not en_intercept_exists)
    return ATTACKER;

  if (role == AGGRESSIVE_DEFENDER and
      this->t_attack_aggro.exists and
      not this->is_duplicate_target(this->t_attack_aggro.position,
                                    this->RULES.BALL_RADIUS))
    return AGGRESSIVE_DEFENDER;

  if (not this->is_duplicate_target(this->t_cross.position,
                                    this->RULES.BALL_RADIUS))
    return GOALKEEPER;

  double dist_to_block = (this->t_block.position - this->me->position.drop()).len();
  double dist_to_follow = (this->t_follow.position - this->me->position.drop()).len();
  if (this->t_block.exists and
      dist_to_block < dist_to_follow /*and
      not this->is_duplicate_target(this->t_block.position)*/)
    return BLOCKER;

  return FOLLOWER;
}

void MyStrategy::set_action(
    model::Action &action,
    const int &id,
    const Vec3D &target_position,
    const Vec3D &target_velocity,
    const double &jump_speed,
    const bool &use_nitro) {
  action.target_velocity_x = target_velocity.x;
  action.target_velocity_y = target_velocity.y;
  action.target_velocity_z = target_velocity.z;
  action.jump_speed = jump_speed;
  action.use_nitro = use_nitro;

  this->robots[id].target_position = target_position;
  this->robots[id].target_velocity = target_velocity;
  this->robots[id].jump_speed = jump_speed;
}


Target MyStrategy::calc_intercept_spot(
    const double &reachable_height,
    const bool &to_shift_x,
    const double &z_offset,
    const double &min_speed,
    const double &max_speed) {
  Vec2D target_position;
  Vec2D target_velocity;
  for (const PosVelTime &ball_pvt : this->ball.projected_path) {
    if (this->sim.goal_scored(ball_pvt.position.z))
      break;
    if (ball_pvt.position.y > reachable_height)
      continue;
    if (ball_pvt.time < BIG_EPS)
      continue;

    // auto [direct_target, scoring_targets] =
    //   this->calc_reachable_targets_from(
    //     this->me->strip(),
    //     this->ball.strip(),
    //     NUM_RAYS
    //   );
    // if (not scoring_targets.empty())
    //   target_position = scoring_targets[int(scoring_targets.size())/2];
    // else {
    target_position.x = ball_pvt.position.x;
    if (to_shift_x) {
     if (target_position.x < -this->GOAL_EDGE)
       target_position.x -= this->RULES.ROBOT_RADIUS;
     else if (target_position.x > this->GOAL_EDGE)
       target_position.x += this->RULES.ROBOT_RADIUS;
    }
    target_position.z = ball_pvt.position.z - z_offset;
    // }

    // if I'm farther than the target..
    if (this->me->position.z > target_position.z)
      continue;

    Vec2D delta_pos = target_position - this->me->position.drop();
    double need_speed = delta_pos.len() / ball_pvt.time;

    // WARNING: the time needed might be different than ball_pvt.time

    if (min_speed <= need_speed and need_speed <= max_speed) {
      target_velocity = delta_pos.normalize() * need_speed;
      return {true, target_position, target_velocity, ball_pvt.time};
    }
  }
  return {false, Vec2D(), Vec2D(), 0.0};
}

Target MyStrategy::calc_defend_spot() {
  Vec2D target_position(
    clamp(this->ball.projected_path[0].position.x,
          -(this->ARENA.goal_width/2.0-2*this->ARENA.bottom_radius),
          this->ARENA.goal_width/2.0-2*this->ARENA.bottom_radius),
    -this->ARENA.depth/2.0-this->RULES.ROBOT_RADIUS);
  Vec2D target_velocity = (target_position - this->me->position.drop()) *
                          this->RULES.ROBOT_MAX_GROUND_SPEED;
  double needed_time = geom::time_to_go_to(
    this->me->position.drop(),
    this->me->velocity.drop(),
    target_position
  );

  auto [me_intercept_exists, i_position, i_time] =
    this->me->first_ball_intercept;
  auto [en_intercept_exists, en_intercept_ball_pos, en_id] =
    this->calc_enemy_first_intercept(i_time);
  // auto [en_locked, en_lock_position] =
  //   geom::ray_circle_first_intersection(
  //     this->robots[en_id].position.drop(),
  //     this->robots[en_id].velocity.drop(),
  //     this->ball.position.drop(),
  //     this->RULES.ROBOT_RADIUS + this->RULES.BALL_RADIUS - BIG_EPS
  //   );

  if (me_intercept_exists and not en_intercept_exists) {
    for (const PosVelTime &ball_pvt : this->ball.projected_path) {
      if (this->sim.goal_scored(ball_pvt.position.z))
        break;
      if (ball_pvt.time < BIG_EPS)
        continue;
      if (ball_pvt.position.z <= this->CRITICAL_BORDER) {
        target_position.x = ball_pvt.position.x;
        Vec2D delta_pos = target_position - this->me->position.drop();
        double need_speed = delta_pos.len() / ball_pvt.time;
        // need_speed = clamp(need_speed,
        //                    0.5*this->RULES.ROBOT_MAX_GROUND_SPEED,
        //                    this->RULES.ROBOT_MAX_GROUND_SPEED);
        target_velocity = delta_pos.normalize() * need_speed;
        // target_velocity = delta_pos.normalize() * this->RULES.ROBOT_MAX_GROUND_SPEED;
        return {true, target_position, target_velocity, ball_pvt.time};
      }
    }
  } /* else if (en_intercept_exists and en_locked) {
    EntityLite r_dummy = this->robots[en_id].lighten();
    r_dummy.position = Vec3D(en_lock_position, this->ball.position.y);
    r_dummy.velocity = Vec3D(en_lock_position - this->robots[en_id].position.drop(), 0).normalize() *
                       this->RULES.ROBOT_MAX_GROUND_SPEED;
    EntityLite b_dummy = this->ball.lighten();

    bool has_collided = this->sim.collide_entities(r_dummy, b_dummy);
    auto [en_can_score, intersection] = geom::ray_segment_intersection(
      b_dummy.position.drop(),
      b_dummy.velocity.drop(),
      Vec2D(-this->GOAL_EDGE, -this->ARENA.depth/2.0),
      Vec2D( this->GOAL_EDGE, -this->ARENA.depth/2.0));

    assert(has_collided);
    if (en_can_score) {
      target_position = en_lock_position;
      Vec2D delta_pos = target_position - this->me->position.drop();
      double need_speed = this->RULES.ROBOT_MAX_GROUND_SPEED;
      double needed_time =
        geom::time_to_go_to(
          this->me->position.drop(),
          this->me->velocity.drop(),
          target_position
        );
      target_velocity = delta_pos.normalize() * need_speed;
      return {true, target_position, target_velocity, needed_time};
    }
  }*/

  return {true, target_position, target_velocity, needed_time};
}

Target MyStrategy::calc_block_spot(const double &offset) {
  int nearest_id = -1;
  Vec2D en_attack_pos;
  double en_attack_time = INF;

  for (int id : this->enemy_ids) {
    auto [exists, first_reachable, time] = this->robots[id].first_ball_intercept;

    if (exists and
        time < en_attack_time and
        this->robots[id].position.z > first_reachable.z) {
      nearest_id = id;
      en_attack_pos = first_reachable.drop();
      en_attack_time = time;
    }
  }

  if (nearest_id == -1)
    return {false, Vec2D(), Vec2D()};

  if (VERBOSITY >= 1)
    this->renderer.draw_sphere(Vec3D(en_attack_pos, 0.0), 1, VIOLET, 1);

  Vec2D target_position = geom::offset_to(
    en_attack_pos,
    this->robots[nearest_id].position.drop(),
    offset,
    true);
  Vec2D target_velocity = (target_position - this->me->position.drop()) *
                          this->RULES.ROBOT_MAX_GROUND_SPEED;
  double needed_time = geom::time_to_go_to(
    this->me->position.drop(),
    this->me->velocity.drop(),
    target_position
  );

  return {
    this->robots[nearest_id].type == ENEMY,
    target_position,
    target_velocity,
    needed_time
  };
}

Target MyStrategy::calc_follow_spot(const double &z_offset) {
  Vec2D target_position;
  Vec2D target_velocity;

  auto [exists, first_reachable, time] = this->me->first_ball_intercept;

  if (exists)
    target_position = first_reachable.drop();
  else
    target_position = this->ball.bounce_positions[0].drop();

  if (VERBOSITY >= 1)
    this->renderer.draw_sphere(Vec3D(target_position, 0.0), 1, WHITE, 1);

  if (target_position.x < -this->GOAL_EDGE)
    target_position.x -= this->RULES.ROBOT_RADIUS;
  else if (target_position.x > this->GOAL_EDGE)
    target_position.x += this->RULES.ROBOT_RADIUS;
  target_position.z -= z_offset;
  target_velocity = (target_position - this->me->position.drop()) *
                          this->RULES.ROBOT_MAX_GROUND_SPEED;
  double needed_time = geom::time_to_go_to(
    this->me->position.drop(),
    this->me->velocity.drop(),
    target_position
  );

  return {true, target_position, target_velocity, needed_time};
}

/*
std::tuple<Vec2D&, std::vector<Vec2D>&> MyStrategy::calc_reachable_targets_from(
    const PosVelTime &robot_pvt,
    const PosVelTime &ball_pvt,
    const int &num_rays) {

  Vec2D direct_target = geom::offset_to(
      ball_pvt.position.drop(),
      robot_pvt.position.drop(),
      3 - BIG_EPS);
  std::vector<Vec2D> scoring_targets;

  // draw_targets
  // this->renderer.draw_sphere(
  //   Vec3D(direct_target, ball_pvt.position.y), 1, BLACK, 1);

  std::vector<Vec2D> tangents =
    geom::get_tangents_to_circle(
      ball_pvt.position.drop(),
      this->RULES.BALL_RADIUS + this->RULES.ROBOT_RADIUS - BIG_EPS,
      robot_pvt.position.drop()
    );

  std::vector<Vec2D> edges;
  if (int(tangents.size()) == 0)
    return std::forward_as_tuple(direct_target, scoring_targets);
  else if (int(tangents.size()) == 1)
    edges = tangents;
  else if (int(tangents.size()) == 2) {
    Vec2D dir_tangents = tangents[1] - tangents[0];
    for (int raw = 0; raw <= num_rays; ++raw) {
      Vec2D in_point = tangents[0] + dir_tangents * (1.0*raw/num_rays);
      Vec2D edge = geom::get_segment_circle_intersection(
        ball_pvt.position.drop(),
        this->RULES.BALL_RADIUS + this->RULES.ROBOT_RADIUS - BIG_EPS,
        in_point,
        robot_pvt.position.drop());
      edges.push_back(edge);
    }
  } else
    assert(false);        // shouldn't happen


  for (const Vec2D &edge : edges) {
    EntityLite r_dummy = this->me->lighten();
    r_dummy.position = Vec3D(edge, ball_pvt.position.y);
    r_dummy.velocity = Vec3D(edge - robot_pvt.position.drop(), 0) *
                       this->RULES.ROBOT_MAX_GROUND_SPEED;
    r_dummy.velocity.clamp(this->RULES.ROBOT_MAX_GROUND_SPEED);
    EntityLite b_dummy = this->ball.lighten();
    b_dummy.position = ball_pvt.position;
    b_dummy.velocity = ball_pvt.velocity;

    bool has_collided = this->sim.collide_entities(r_dummy, b_dummy);
    auto [can_score, intersection] = geom::ray_segment_intersection(
      b_dummy.position.drop(),
      b_dummy.velocity.drop(),
      Vec2D(-this->GOAL_EDGE, this->ARENA.depth/2.0),
      Vec2D( this->GOAL_EDGE, this->ARENA.depth/2.0));

    assert(has_collided);
    if (can_score)
      scoring_targets.push_back(edge);

    if (VERBOSITY >= 1) {
      this->renderer.draw_sphere(
        r_dummy.position,
        1.0,
        (can_score ? VIOLET : TEAL),
        0.5
      );
      this->renderer.draw_line(
        r_dummy.position,
        this->me->position,
        1.0,
        (can_score ? VIOLET : TEAL),
        0.5
      );
      this->renderer.draw_line(
        b_dummy.position,
        Vec3D(intersection, 0.0),
        1.0,
        (can_score ? VIOLET : TEAL),
        0.5
      );
    }
  }

  return std::forward_as_tuple(direct_target, scoring_targets);
}
*/

bool MyStrategy::is_duplicate_target(
    const Vec2D &position,
    const double &acceptable_delta) {
  double dist_to_target = (position - this->me->position.drop()).len();

  for (int id : this->ally_ids) {
    if (id == this->me_id)  continue;
    double other_dist_to_target = (position - this->robots[id].position.drop()).len();

    double delta_targets = (this->robots[id].target_position.drop() - position).len();
    if (delta_targets < acceptable_delta and
        other_dist_to_target < dist_to_target)
      return true;

    // if (is_attacker(roles[robot.id]) and is_attacker(roles[id]) and
    //     delta_pos_other.len() < delta_pos.len()) {
    //   return true;
    // }
  }
  return false;
}

std::tuple<bool, Vec3D, int> MyStrategy::calc_enemy_first_intercept(
    const double &until) {
  double intercept_time = INF;
  Vec3D intercept_pos;
  int intercept_id = -1;
  for (int id : this->enemy_ids) {
    auto [exists, position, time] = this->robots[id].first_ball_intercept;
    if (exists and time < until and time < intercept_time) {
      intercept_time = time;
      intercept_pos = position;
      intercept_id = id;
    }
  }
  if (intercept_time < until)
    return {true, intercept_pos, intercept_id};
  return {false, intercept_pos, intercept_id};
}

double MyStrategy::calc_jump_speed(const double &acceptable_jump_dist) {
  auto [exists, ball_pos, robot_pos] = calc_valid_jump_intercept(
     this->me->projected_jump_path,
     this->ball.projected_path,
     this->me->position);

  if (not exists)
    return 0.0;

  if (this->me->role == AGGRESSIVE_DEFENDER and
      this->me->velocity.len() > this->RULES.ROBOT_MAX_GROUND_SPEED-1)
    return this->RULES.ROBOT_MAX_JUMP_SPEED;

  if (this->me->position.z < robot_pos.z and
      (ball_pos - robot_pos).len() <= acceptable_jump_dist)
    return this->RULES.ROBOT_MAX_JUMP_SPEED;

  return 0.0;
}

std::tuple<bool, Vec3D, Vec3D> MyStrategy::calc_valid_jump_intercept(
    const Path &robot_path,
    const Path &ball_path,
    const Vec3D &robot_position) {
  double prev_max_height = -INF;
  for (int i = 0; i < std::min(int(robot_path.size()), int(ball_path.size())); ++i) {
    assert(std::fabs(robot_path[i].time - ball_path[i].time) < BIG_EPS);
    if ((ball_path[i].position - robot_path[i].position).len() <= this->RULES.BALL_RADIUS + this->RULES.ROBOT_RADIUS) {
      if (ball_path[i].position.z >= robot_position.z and
          ball_path[i].position.z >= robot_path[i].position.z + 0.5 and
          ball_path[i].position.y >= robot_path[i].position.y and
          robot_path[i].position.y >= prev_max_height) {
        return std::forward_as_tuple(true, ball_path[i].position, robot_path[i].position);
      } else
        return std::forward_as_tuple(false, Vec3D(), Vec3D());
    }
    prev_max_height = std::max(prev_max_height, robot_path[i].position.y);
  }
  return std::forward_as_tuple(false, Vec3D(), Vec3D());
}


std::string MyStrategy::custom_rendering() {
  if (VERBOSITY >= 1) {
    // draw borders
    this->renderer.draw_border(this->DEFENSE_BORDER);
    this->renderer.draw_border(this->CRITICAL_BORDER);

    // draw goal
    this->renderer.draw_line(
      Vec3D(this->GOAL_LIM_LEFT, 1.0),
      Vec3D(this->GOAL_LIM_RIGHT, 1.0),
      10,
      RED,
      0.5);
    this->renderer.draw_sphere(Vec3D(this->GOAL_LIM_LEFT, 1.0), 1, RED, 1);
    this->renderer.draw_sphere(Vec3D(this->GOAL_LIM_RIGHT, 1.0), 1, RED, 1);

    // // draw supersized ball
    // this->renderer.draw_sphere(this->ball.position, 3, WHITE, 0.1);

    // draw ball path
    this->renderer.draw_path(this->ball.projected_path, 0.5, RED, 0.5);
    for (Vec3D position : this->ball.bounce_positions)
      this->renderer.draw_sphere(position, 1, BLACK, 1.0);

    // draw jump paths of the robots
    // for (auto &[id, robot] : this->robots)
    //   this->renderer.draw_path(robot.projected_jump_path, 0.5, YELLOW, 0.5, false);

    for (auto &[id, robot] : this->robots)
      this->renderer.draw_line(
        robot.position,
        Vec3D(robot.position.x, robot.position.z, ARENA.height),
        10,
        (robot.position.y > RULES.ROBOT_RADIUS ? YELLOW : TEAL),
        0.5);

    for (auto &[id, robot] : this->robots)
      this->renderer.draw_path(robot.projected_path, 0.5, TEAL, 0.5, false);


    for (int id : this->ally_ids) {
      Vec3D hover = this->robots[id].position;
      hover.y += 1.5*this->RULES.ROBOT_RADIUS;
      this->renderer.draw_sphere(
        hover,
        0.5,
        ColorMap[this->robots[id].role],
        1.0);
      this->renderer.draw_sphere(
        this->robots[id].target_position,
        1.0,
        ColorMap[this->robots[id].role],
        0.5);
      this->renderer.draw_line(
        hover,
        this->robots[id].target_position,
        10,
        ColorMap[this->robots[id].role],
        0.5
      );
    }

    for (int id : this->robot_ids) {
      auto [exists, final_pos, t_flight] =
        geom::calc_flight(
          this->robots[id].position,
          this->robots[id].velocity,
          -this->RULES.GRAVITY);
      if (exists)
        this->renderer.draw_sphere(
          Vec3D(final_pos, 0.0),
          1.0,
          BLACK,
          0.5);
    }

    // if (VERBOSITY >= 2)
    //   std::cout<<this->t_attack_aggro.needed_time<<"\n";
    // for (int id : this->robot_ids) {
    //   auto [exists, target, time] = this->robots[id].first_ball_intercept;
    //   if (exists)
    //     this->renderer.draw_sphere(
    //       target,
    //       2.5,
    //       VIOLET,
    //       0.5);
    //   if (VERBOSITY >= 2)
    //     std::cout<<id<<" "<<time<<"\n";
    // }
    for (int id : this->enemy_ids) {
      auto [exists, position] =
        geom::ray_circle_first_intersection(
          this->robots[id].position.drop(),
          this->robots[id].velocity.drop(),
          this->ball.position.drop(),
          this->RULES.ROBOT_RADIUS + this->RULES.BALL_RADIUS
        );
      if (exists) {
        this->renderer.draw_sphere(
          Vec3D(position, this->ball.position.y),
          1.0,
          VIOLET,
          1.0
        );
        this->renderer.draw_line(
          this->robots[id].position,
          Vec3D(position, this->ball.position.y),
          10,
          VIOLET,
          1.0
        );
      }
    }
  }

  return this->renderer.get_json();
}

#endif
