#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif

#ifndef _MY_STRATEGY_CPP_
#define _MY_STRATEGY_CPP_

#include "MyStrategy.h"

using namespace model;

MyStrategy::MyStrategy() { }

void MyStrategy::act(
    const Robot& me,
    const Rules& rules,
    const Game& game,
    Action& action) {

  if (game.current_tick == 0 and not this->initialized)
    this->init_strategy(rules, game);

  if (this->prev_tick != game.current_tick) {
    if (game.current_tick % 100 == 0)
      std::cout<<game.current_tick<<"\n";
    renderer.clear();
    this->prev_tick = game.current_tick;
  }

  Vec2D my_position_2d = {me.x, me.z};
  Vec3D my_position_3d = {my_position_2d, me.y};
  Vec2D ball_position_2d = {game.ball.x, game.ball.z};
  Vec3D ball_position_3d = {ball_position_2d, game.ball.y};

  this->robots = std::vector<Robot>(int(game.robots.size())+1);
  for (Robot robot : game.robots)
    this->robots[robot.id] = robot;
  this->robot_positions[me.id] = my_position_3d;
  this->robot_velocities[me.id] = {me.velocity_x, me.velocity_z, me.velocity_y};

  this->run_simulation(game);

  attack = calc_intercept_spot(
    projected_ball_path,
    my_position_2d,
    4*rules.ROBOT_RADIUS,
    true);
  attack_aggro = calc_intercept_spot(
    projected_ball_path,
    my_position_2d,
    6*rules.ROBOT_RADIUS,
    false);
  cross = calc_defend_spot(projected_ball_path, my_position_2d);
  default_strat = get_default_strat(my_position_2d, ball_position_2d);

  roles[me.id] = calc_role(
    me.id,
    my_position_3d,
    ball_position_3d);


  Vec2D target_position;
  Vec2D target_velocity;

  switch (roles[me.id]) {
    case ATTACKER:
    case AGGRESSIVE_DEFENDER:
      target_position = attack.position;
      target_velocity = attack.velocity;
      break;
    case DEFENDER:
      target_position = cross.position;
      target_velocity = cross.velocity;
      break;
    case DEFAULT:
      target_position = default_strat.position;
      target_velocity = default_strat.velocity;
      break;
    default:
      break;
  };

  if (not me.touch and me.velocity_y < 0) {
    set_action(
      action,
      me.id,
      Vec3D(my_position_2d, 0.0),
      Vec3D(0.0, 0.0, -rules.ROBOT_MAX_GROUND_SPEED),
      0.0,
      true
    );
  } else {
    double jump_speed = calc_jump_speed(
      my_position_3d,
      ball_position_3d,
      Vec3D(game.ball.velocity_x, game.ball.velocity_z, game.ball.velocity_z),
      me.id);
    set_action(
      action,
      me.id,
      Vec3D(target_position, 0.0),
      Vec3D(target_velocity, 0.0),
      jump_speed,
      (std::fabs(rules.ROBOT_MAX_GROUND_SPEED -
       this->robot_velocities[me.id].len()) > EPS and
       jump_speed > EPS)
    );
  }
  /*set_action(
    action,
    me.id,
    Vec3D(0.0, 0.0, 20.0),
    Vec3D(0.0, rules.ROBOT_MAX_GROUND_SPEED, 0.0),
    rules.ROBOT_MAX_JUMP_SPEED,
    false
  );*/
  /*
  std::cout<<"current_tick: "<<game.current_tick<<"\n";
  std::cout<<me.id<<" "<<me.nitro_amount<<"\n";
  */
}

void MyStrategy::init_strategy(
    const model::Rules &rules,
    const model::Game &game) {
  std::cout<<"START!\n";

  this->rules = rules;
  this->arena = rules.arena;
  this->DEFENSE_BORDER = -arena.depth/6.0;
  this->CRITICAL_BORDER = -(arena.depth/2.0 - arena.top_radius);

  // get IDs of allies
  for (Robot robot : game.robots) {
    if (robot.is_teammate)
      this->ally_ids.push_back(robot.id);
    else
      this->enemy_ids.push_back(robot.id);
  }

  // fix target_positions
  for (int i = 0; i < int(game.robots.size()); ++i) {
    this->target_positions.push_back(Vec3D());
    this->target_velocities.push_back(Vec3D());
    this->jump_speeds.push_back(0.0);
    this->robot_positions.push_back(Vec3D());
    this->robot_velocities.push_back(Vec3D());
    this->roles.push_back(DEFAULT);
  }

  // fix predictions
  projected_ball_path = {};
  projected_jump_paths = std::vector<Path>(int(game.robots.size()) + 1);
  projected_robot_paths = std::vector<Path>(int(game.robots.size()) + 1);

  this->sim = Simulation(
    game.ball,
    game.robots,
    rules);

  this->initialized = true;
}

void MyStrategy::run_simulation(const model::Game &game) {
  sim.set(
    game.ball,
    game.robots,
    this->target_velocities,
    this->jump_speeds,
    game.current_tick);

  /*for (int id = 1; id <= int(game.robots.size()); ++id)
    projected_jump_paths[id] = sim.get_jump_path(sim.robots[id]);*/

  sim.run(
      int(SIMULATION_DURATION/SIMULATION_PRECISION),
      SIMULATION_PRECISION);

  ball_bounce_positions = sim.ball_bounce_positions;
  projected_ball_path = sim.proj_ball_path;
  projected_robot_paths = sim.proj_robot_paths;
}

Target MyStrategy::calc_intercept_spot(
    const Path &ball_path,
    const Vec2D &my_position,
    const double &acceptable_height,
    const bool &to_shift_x) {

  Vec2D target_position;
  Vec2D target_velocity;

  for (int i = 1; i < int(ball_path.size()); ++i) {
    if (goal_scored(ball_path[i].z))
      break;

    double t = ball_path[i].t;
    /*
    Vec2D ball_position = {ball_path[i].x, ball_path[i].z};

    double start_x = -(arena.goal_width/2.0-arena.bottom_radius);
    double end_x   =  (arena.goal_width/2.0-arena.bottom_radius);
    double width_x = end_x - start_x;
    const int NUM_GOAL_PARTS = 4;

    std::vector<PositionAndDist> possible_targets;
    for (int part = 0; part <= NUM_GOAL_PARTS; ++part) {
      Vec2D lim = {start_x + ((1.0*part)/NUM_GOAL_PARTS) * width_x, arena.depth/2.0};
      possible_targets.push_back(
        calc_optimal_intercept_target(
          ball_position,
          lim,
          rules.BALL_RADIUS + rules.ROBOT_RADIUS));
    }

    std::sort(possible_targets.begin(), possible_targets.end());

    target_position = possible_targets.back().position;
    */

    target_position.x = ball_path[i].x;
    if (to_shift_x) {
      if (target_position.x < - (arena.goal_width/2.0 - arena.goal_top_radius))
        target_position.x -= rules.ROBOT_RADIUS;
      else if (target_position.x > arena.goal_width/2.0 - arena.goal_top_radius)
        target_position.x += rules.ROBOT_RADIUS;
    }
    target_position.z = ball_path[i].z - 1.5 * rules.ROBOT_RADIUS;

    // If ball will not leave arena boundary
    // (collision with the arena would happen, but we are not considering it),
    // and the ball will be closer to opponent's net than the robot,
    if (target_position.z > my_position.z and
        ball_path[i].y <= acceptable_height) {
      // Compute the speed robot needs to run with
      // To be at ball's location at the same time as the ball
      Vec2D delta_pos = target_position - my_position;

      double need_speed = delta_pos.len() / t;
      // If the speed is in acceptable range
      if (0.5 * rules.ROBOT_MAX_GROUND_SPEED < need_speed and
          need_speed < rules.ROBOT_MAX_GROUND_SPEED) {
        target_velocity = delta_pos.normalize() * need_speed;
        return {true, target_position, target_velocity};
      }
    }
  }
  return {false, Vec2D(), Vec2D()};
}

PositionAndDist MyStrategy::calc_optimal_intercept_target(
    const Vec2D &p1,
    const Vec2D &p2,
    const double &offset) {
  Vec2D target = p1 + (p1 - p2).normalize() * offset;
  double robot_dist = robots_dist_to_line_segment(target, p2);
  /*
  //std::cout<<"target: "<<p2.str()<<" || dist: "<<robot_dist<<"\n";
  renderer.draw_sphere(
      Vec3D(p2, 0.0),
      1,
      VIOLET,
      1.0);
  renderer.draw_line(
      Vec3D(target, 2.0),
      Vec3D(p2, 2.0),
      20,
      VIOLET,
      1);
  */
  return {target, robot_dist};
}

double MyStrategy::robots_dist_to_line_segment(
    const Vec2D &p1,
    const Vec2D &p2) {
  double dist = 1e9;

  Vec2D dir = p2 - p1;
  double hypo_squared = dir.len_sqr();

  for (int id : this->enemy_ids) {
    Vec2D robot_pos(robots[id].x, robots[id].z);

    double factor = (robot_pos - p1).dot(dir) / hypo_squared;
    factor = clamp(factor, 0.0, 1.0);

    Vec2D projection = p1 + dir * factor;

    double dist_to_line = (robot_pos - projection).len();

    dist = std::min(dist, dist_to_line);
  }

  return dist;
}

Target MyStrategy::calc_defend_spot(
    const Path &ball_path,
    const Vec2D &my_position) {

  Vec2D target_position(
    clamp(ball_path[0].x,
          -(arena.goal_width/2.0-2*arena.bottom_radius),
          arena.goal_width/2.0-2*arena.bottom_radius),
    -arena.depth/2.0);
  Vec2D target_velocity = (target_position - my_position) *
                          rules.ROBOT_MAX_GROUND_SPEED;

  for (int i = 1; i < int(ball_path.size()); ++i) {
    if (goal_scored(ball_path[i].z))
      break;

    double t = ball_path[i].t;

    if (ball_path[i].z <= -arena.depth/2.0) {
      target_position.x = ball_path[i].x;
      Vec2D delta_pos = target_position - my_position;
      double need_speed = delta_pos.len() / t;
      target_velocity = delta_pos.normalize() * need_speed;
      return {true, target_position, target_velocity};
    }
  }
  return {true, target_position, target_velocity};
}

Target MyStrategy::get_default_strat(
    const Vec2D &my_position,
    const Vec2D &ball_position) {
  Vec2D target_position;
  Vec2D target_velocity;

  Vec2D first_bounce(ball_bounce_positions[0].x, ball_bounce_positions[0].z);

  int id = get_id_nearest_to(first_bounce);
  if (not this->robots[id].is_teammate and this->robots[id].z > ball_position.z) {
    Vec2D enemy_pos = Vec2D(this->robots[id].x, this->robots[id].z);
    PositionAndDist res = calc_optimal_intercept_target(first_bounce, enemy_pos, 2*rules.BALL_RADIUS);
    target_position = res.position;
  } else {
    target_position = Vec2D(ball_bounce_positions[0].x,
                            ball_bounce_positions[0].z - 2*rules.BALL_RADIUS);
    if (target_position.x < - (arena.goal_width/2.0 - arena.bottom_radius))
      target_position.x -= rules.ROBOT_RADIUS;
    else if (target_position.x > arena.goal_width/2.0 - arena.bottom_radius)
      target_position.x += rules.ROBOT_RADIUS;
  }
  target_velocity = Vec2D(target_position - my_position) *
                           rules.ROBOT_MAX_GROUND_SPEED;
  return {true, target_position, target_velocity};
}

int MyStrategy::get_id_nearest_to(const Vec2D &position) {
  int nearest_id = -1;
  double nearest_dist = 1e9;
  for (int id = 1; id <= int(this->robots.size()); ++id) {
    double dist = (Vec2D(this->robots[id].x, this->robots[id].z) - position).len();
    if (dist < nearest_dist) {
      nearest_id = id;
      nearest_dist = dist;
    }
  }
  return nearest_id;
}

Role MyStrategy::calc_role(
    const int &id,
    const Vec3D &my_position,
    const Vec3D &ball_position) {
  Vec2D my_pos_2d = {my_position.x, my_position.z};

  // The robot is a defender by default
  Role role = (robots.size() == 2 ? ATTACKER : DEFENDER);

  // The robot will be an attacker if there already is a robot closer to the net
  // than the current one.
  for (int ally_id : ally_ids)
    if (ally_id != id and robot_positions[ally_id].z < my_position.z)
      role = ATTACKER;

  double dist_to_ball = (my_position -
                         Vec3D(attack_aggro.position, rules.ROBOT_RADIUS)).len();
  bool is_closer_to_ball_than_enemies = true;
  for (Robot robot : robots) {
    if (not robot.is_teammate) {
      double other_dist_to_ball = (Vec3D(robot.x, robot.z, robot.y) -
                                   Vec3D(attack_aggro.position, rules.ROBOT_RADIUS)).len();
      if (other_dist_to_ball < dist_to_ball)
        is_closer_to_ball_than_enemies = false;
    }
  }
  if (role == DEFENDER and
      attack_aggro.exists and
      attack_aggro.position.z <= DEFENSE_BORDER and
      is_closer_to_ball_than_enemies) {
    role = AGGRESSIVE_DEFENDER;
  }

  if (attack.exists and is_attacker(role))
    if (!is_duplicate_target(attack.position, my_pos_2d, id))
      return role;

  if (!is_duplicate_target(cross.position, my_pos_2d, id))
    return DEFENDER;

  return DEFAULT;
}

double MyStrategy::calc_jump_speed(
    const Vec3D &my_position,
    const Vec3D &ball_position,
    const Vec3D &ball_velocity,
    const int &id) {
  double dist_to_ball = (my_position - ball_position).len();
  double acceptable_dist = rules.BALL_RADIUS + 6*rules.ROBOT_MAX_RADIUS;

  Role role = roles[id];

  Entity en_attack = {
    robot_positions[id],
    robot_velocities[id],
    Vec3D(attack.velocity, 0.0),
    rules.ROBOT_RADIUS,
    0.0,
    rules.ROBOT_MASS,
    rules.ROBOT_ARENA_E,
    ALLY,
    id
  };
  Entity en_attack_aggro = {
    robot_positions[id],
    robot_velocities[id],
    Vec3D(attack_aggro.velocity, 0.0),
    rules.ROBOT_RADIUS,
    0.0,
    rules.ROBOT_MASS,
    rules.ROBOT_ARENA_E,
    ALLY,
    id
  };
  /*
  Entity en_ball {
    ball_position,
    ball_velocity,
    Vec3D(),
    rules.BALL_RADIUS,
    0.0,
    rules.BALL_MASS,
    rules.BALL_ARENA_E,
    BALL,
    -1
  };

  if (role == ATTACKER) {
    JumpTimePQ possible_jump_speeds;
    possible_jump_speeds.push({1e9, 0.0});
    JumpTimePQ scoring_jump_speeds;

    const int NUM_PARTS = 5;
    for (int part = 0; part <= NUM_PARTS; ++part) {
      double jump_speed = (1.0 * part / NUM_PARTS) * rules.ROBOT_MAX_JUMP_SPEED;
      JumpBallIntercept intercept = sim.simulate_jump(
          en_attack,
          en_ball,
          SIMULATION_PRECISION,
          jump_speed);

      renderer.draw_ball_path(intercept.robot_path, 1, BLACK, 0.25);

      if (not intercept.exists)
        continue;
      if (intercept.robot_pos.z < my_position.z or
          intercept.ball_pos.z < intercept.robot_pos.z or
          intercept.ball_pos.y < intercept.robot_pos.y)
        continue;

      projected_jump_paths[id] = intercept.robot_path;
      speculative_ball_path = intercept.ball_path;

      possible_jump_speeds.push({intercept.robot_pos.t, jump_speed});
      if (intercept.can_score)
        scoring_jump_speeds.push({intercept.robot_pos.t, jump_speed});
    }

    if (not scoring_jump_speeds.empty())
      return scoring_jump_speeds.top().second;
    if (my_position.z < ball_position.z and dist_to_ball < acceptable_dist)
      return possible_jump_speeds.top().second;
  }
  */

  if (role == AGGRESSIVE_DEFENDER) {
    Path jump_path = sim.get_jump_path(
        en_attack_aggro,
        SIMULATION_PRECISION,
        rules.ROBOT_MAX_JUMP_SPEED);
    projected_jump_paths[id] = jump_path;

    TargetJump ball_intercept = calc_jump_intercept(
       jump_path,
       projected_ball_path,
       my_position);

    if (not ball_intercept.exists)
      return 0.0;

    if (robot_velocities[id].len() > rules.ROBOT_MAX_GROUND_SPEED-1)
      return rules.ROBOT_MAX_JUMP_SPEED;
  } else {
    Path jump_path = sim.get_jump_path(
        en_attack,
        SIMULATION_PRECISION,
        rules.ROBOT_MAX_JUMP_SPEED);
    projected_jump_paths[id] = jump_path;

    TargetJump ball_intercept = calc_jump_intercept(
       jump_path,
       projected_ball_path,
       my_position);

    if (not ball_intercept.exists)
      return 0.0;

    if ((role == DEFENDER and ball_intercept.robot_pos.z <= CRITICAL_BORDER) or
        role == DEFAULT)
      return rules.ROBOT_MAX_JUMP_SPEED;
  }

  // default behavior
  if (my_position.z < ball_position.z and dist_to_ball < acceptable_dist)
    return rules.ROBOT_MAX_JUMP_SPEED;
  return 0.0;
}

bool MyStrategy::goal_scored(double z) {
  return std::fabs(z) > arena.depth/2.0 + rules.BALL_RADIUS;
}

TargetJump MyStrategy::calc_jump_intercept(
    const Path &robot_path,
    const Path &ball_path,
    const Vec3D &my_position) {
  double prev_max_height = -1e9;
  for (int i = 0; i < std::min(int(robot_path.size()), int(ball_path.size())); ++i) {
    assert(std::fabs(robot_path[i].t - ball_path[i].t) < EPS);
    if ((ball_path[i] - robot_path[i]).len() <= rules.BALL_RADIUS + rules.ROBOT_RADIUS) {
      if (ball_path[i].z >= my_position.z and
          ball_path[i].z >= robot_path[i].z + 0.5 and
          ball_path[i].y >= robot_path[i].y and
          robot_path[i].y >= prev_max_height) {
        return {true, ball_path[i], robot_path[i]};
      } else
        return {false, Vec3D(), Vec3D()};
    }
    prev_max_height = std::max(prev_max_height, robot_path[i].y);
  }
  return {false, Vec3D(), Vec3D()};
}

bool MyStrategy::is_duplicate_target(
    const Vec2D &target_position,
    const Vec2D &my_position,
    const int &id) {
  Vec2D delta_pos = target_position - my_position;

  for (Robot robot : robots) {
    if (!robot.is_teammate or robot.id == id)
      continue;

    Vec2D target_pos_other = Vec2D(
      target_positions[robot.id].x,
      target_positions[robot.id].z
    );
    Vec2D delta_pos_other = target_pos_other - Vec2D(robot.x, robot.z);

    if ((target_position - target_pos_other).len() < rules.BALL_RADIUS and
        delta_pos_other.len() < delta_pos.len()) {
      return true;
    }

    if (is_attacker(roles[robot.id]) and is_attacker(roles[id]) and
        delta_pos_other.len() < delta_pos.len()) {
      return true;
    }
  }
  return false;
}

bool MyStrategy::is_attacker(const Role &role) {
  return role == ATTACKER or
         role == AGGRESSIVE_DEFENDER;
}

bool MyStrategy::is_defender(const Role &role) {
  return !is_attacker(role);
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

  target_positions[id] = target_position;
  target_velocities[id] = target_velocity;
  jump_speeds[id] = jump_speed;
}

std::string MyStrategy::custom_rendering() {
  // draw borders
  renderer.draw_border(DEFENSE_BORDER);
  renderer.draw_border(CRITICAL_BORDER);

  // draw ball bounce positions
  for (Vec3D pos : ball_bounce_positions) {
    pos.y = 0;
    renderer.draw_sphere(pos, 1, BLACK, 1);
  }

  // predicted paths of the ball
  renderer.draw_ball_path(projected_ball_path, 0.5, RED, 0.25);
  renderer.draw_ball_path(speculative_ball_path, 0.5, VIOLET, 0.25);

  // position of the robots
  for (int id = 1; id <= 2; ++id) {
    Vec3D start_pos = robot_positions[id];
    if (start_pos.y > rules.ROBOT_RADIUS)
      renderer.draw_line(start_pos, {start_pos.x, start_pos.z, 20}, 10, YELLOW, 0.5);
    else
      renderer.draw_line(start_pos, {start_pos.x, start_pos.z, 20}, 10, TEAL, 0.5);
  }

  /*
  // predicted paths of the robots
  for (int id = 1; id < int(projected_robot_paths.size()); ++id) {
    for (int i = 1; i < int(projected_robot_paths[id].size()); ++i) {
      if (2*i <= SIMULATION_DURATION/SIMULATION_PRECISION) {
        Vec3D prev_pos = projected_robot_paths[id][i-1];
        Vec3D position = projected_robot_paths[id][i];
        renderer.draw_line(prev_pos, position, 10, TEAL, 0.5);
      }
    }
  }
  */

  // predicted jump paths of the robots
  for (int id = 1; id < int(projected_jump_paths.size()); ++id) {
    for (int i = 1; i < int(projected_jump_paths[id].size()); ++i) {
      Vec3D position = projected_jump_paths[id][i];
      renderer.draw_sphere(position, 0.5, YELLOW, 0.5);
    }
  }

  /*
  // predicted defense paths of the robots
  Entity en = {
    Vec3D(arena.goal_width/2.0 - arena.bottom_radius, -arena.depth/2.0, 1.0),
    Vec3D(0.0, 0.0, 0.0),
    Vec3D(-rules.ROBOT_MAX_GROUND_SPEED, 0.0, rules.ROBOT_MAX_GROUND_SPEED),
    rules.ROBOT_RADIUS,
    0.0,
    rules.ROBOT_MASS,
    rules.ROBOT_ARENA_E,
    ALLY,
    -1};
  Entity enc = en;
  Path def_path = sim.get_defence_path(en, 60, rules.ROBOT_MAX_JUMP_SPEED);
  Path def_path_2 = sim.get_defence_path(enc, 60, 0.5*rules.ROBOT_MAX_JUMP_SPEED);
  for (Vec3D pos : def_path)
    renderer.draw_sphere(pos, 1, BLACK, 0.5);
  for (Vec3D pos : def_path_2)
    renderer.draw_sphere(pos, 1, BLACK, 0.5);
  */

  // roles of the robots
  for (int id : ally_ids) {
    Vec3D hover = robot_positions[id];
    hover.y += 1.5*rules.ROBOT_RADIUS;
    switch(roles[id]) {
      case ATTACKER:
        renderer.draw_sphere(hover,                0.5, RED, 1.0);
        renderer.draw_sphere(target_positions[id], 1.0, RED, 0.5);
        break;
      case AGGRESSIVE_DEFENDER:
        renderer.draw_sphere(hover,                0.5, LIGHT_RED, 1.0);
        renderer.draw_sphere(target_positions[id], 1.0, LIGHT_RED, 0.5);
        break;
      case DEFENDER:
        renderer.draw_sphere(target_positions[id], 1.0, BLUE, 0.5);
        renderer.draw_sphere(hover,                0.5, BLUE, 1.0);
        break;
      case DEFAULT:
        renderer.draw_sphere(hover,                0.5, WHITE, 1.0);
        renderer.draw_sphere(target_positions[id], 1.0, WHITE, 0.5);
        break;
      default:
        renderer.draw_sphere(hover,                0.5, BLACK, 1.0);
        renderer.draw_sphere(target_positions[id], 1.0, BLACK, 0.5);
    };
  }

  return renderer.get_json();
}

#endif
