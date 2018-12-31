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

  Vec2D my_position_2d = {me.x, me.z};
  Vec3D my_position_3d = {my_position_2d, me.y};
  Vec2D ball_position_2d = {game.ball.x, game.ball.z};
  Vec3D ball_position_3d = {ball_position_2d, game.ball.y};

  this->robot_positions[me.id] = my_position_3d;
  this->robot_velocities[me.id] = {me.velocity_x, me.velocity_z, me.velocity_y};

  if (this->prev_tick != game.current_tick and game.current_tick % 100 == 0)
    std::cout<<game.current_tick<<"\n";
  this->prev_tick = game.current_tick;

  this->run_simulation(game);

  Target attack = calc_intercept_spot(
    projected_ball_path,
    my_position_2d,
    4*rules.ROBOT_RADIUS,
    true);
  Target attack_aggro = calc_intercept_spot(
    projected_ball_path,
    my_position_2d,
    6*rules.ROBOT_RADIUS,
    false);
  Target attack_spec = calc_intercept_spot(
    projected_ball_spec_path,
    my_position_2d,
    4*rules.ROBOT_RADIUS,
    false,
    true,
    projected_ball_path);
  Target cross = calc_defend_spot(projected_ball_path, my_position_2d);
  Target cross_spec = calc_defend_spot(projected_ball_spec_path, my_position_2d);
  Target default_strat = get_default_strat(my_position_2d, ball_position_2d);

  roles[me.id] = calc_role(
    me.id,
    my_position_3d,
    ball_position_3d,
    game.robots,
    attack,
    attack_aggro,
    attack_spec,
    cross,
    cross_spec);


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
    case SPECULATIVE_DEFENDER:
      target_position = cross_spec.position;
      target_velocity = cross_spec.velocity;
      break;
    case SPECULATIVE_ATTACKER:
      target_position = attack_spec.position;
      target_velocity = attack_spec.velocity;
      break;
    case DEFAULT:
      target_position = default_strat.position;
      target_velocity = default_strat.velocity;
      break;
    default:
      break;
  };

  set_action(
    action,
    me.id,
    Vec3D(target_position, 0.0),
    Vec3D(target_velocity, 0.0),
    calc_jump_speed(my_position_3d, ball_position_3d, me.id),
    false
  );
  /*set_action(
    action,
    me.id,
    Vec3D(0.0, 0.0, 20.0),
    Vec3D(0.0, rules.ROBOT_MAX_GROUND_SPEED, 0.0),
    rules.ROBOT_MAX_JUMP_SPEED,
    false
  );*/
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
  for (Robot robot : game.robots)
    if (robot.is_teammate)
      this->ally_ids.push_back(robot.id);

  // fix target_positions
  for (int i = 0; i < int(game.robots.size()); ++i) {
    this->target_positions.push_back(Vec3D());
    this->target_velocities.push_back(Vec3D());
    this->jump_speeds.push_back(0.0);
    this->robot_positions.push_back(Vec3D());
    this->robot_velocities.push_back(Vec3D());
    this->roles.push_back(DEFAULT);
  }

  this->sim = Simulation(
    game.ball,
    game.robots,
    rules,
    SIMULATION_PRECISION);

  this->initialized = true;
}

void MyStrategy::run_simulation(const model::Game &game) {
  sim.update(
    game.ball,
    game.robots,
    this->target_velocities,
    this->jump_speeds,
    game.current_tick);

  projected_jump_paths = std::vector<Path>(int(game.robots.size()) + 1);
  for (int id = 1; id <= int(game.robots.size()); ++id)
    projected_jump_paths[id] = sim.get_jump_path(sim.robots[id]);

  sim.run(int(SIMULATION_DURATION/SIMULATION_PRECISION));

  projected_ball_path = sim.proj_ball_path;
  projected_ball_spec_path = sim.proj_ball_spec_path;
  projected_robot_paths = sim.proj_robot_paths;
}

Target MyStrategy::calc_intercept_spot(
    const Path &ball_path,
    const Vec2D &my_position,
    const double &acceptable_height,
    const bool &to_shift_x,
    const bool &is_speculative,
    const Path &avoid_path) {

  Vec2D target_position;
  Vec2D target_velocity;

  for (int i = 1; i < int(ball_path.size()); ++i) {
    if (goal_scored(ball_path[i].z))
      break;
    if (is_speculative and (ball_path[i] - avoid_path[i]).len() < rules.BALL_RADIUS)
      continue;

    double t = i * SIMULATION_PRECISION;

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

Target MyStrategy::calc_defend_spot(
    const Path &ball_path,
    const Vec2D &my_position) {

  Vec2D target_position(
    sim.clamp(ball_path[0].x,
              -(arena.goal_width/2.0-2*arena.bottom_radius),
              arena.goal_width/2.0-2*arena.bottom_radius),
    -arena.depth/2.0);
  Vec2D target_velocity = (target_position - my_position) *
                          rules.ROBOT_MAX_GROUND_SPEED;

  for (int i = 1; i < int(ball_path.size()); ++i) {
    if (goal_scored(ball_path[i].z))
      break;

    double t = i * SIMULATION_PRECISION;

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
  Vec2D target_position(ball_position.x, ball_position.z - 2*rules.BALL_RADIUS);
  Vec2D target_velocity = (target_position - my_position) *
                           rules.ROBOT_MAX_GROUND_SPEED;
  return {true, target_position, target_velocity};
}

Role MyStrategy::calc_role(
    const int &id,
    const Vec3D &my_position,
    const Vec3D &ball_position,
    const std::vector<Robot> &robots,
    const Target &attack,
    const Target &attack_aggro,
    const Target &attack_spec,
    const Target &cross,
    const Target &cross_spec) {
  Vec2D my_pos_2d = {my_position.x, my_position.z};

  // The robot is a defender by default
  Role role = (robots.size() == 2 ? ATTACKER : DEFENDER);

  // The robot will be an attacker if there already is a robot closer to the net
  // than the current one.
  for (int ally_id : ally_ids)
    if (ally_id != id and robot_positions[ally_id].z < my_position.z)
      role = ATTACKER;

  if (role == DEFENDER and
      attack_aggro.exists and
      attack_aggro.position.z <= DEFENSE_BORDER)
    role = AGGRESSIVE_DEFENDER;

  if (attack.exists and is_attacker(role))
    if (!is_duplicate_target(attack.position, my_pos_2d, id, robots))
      return role;

  if (!is_duplicate_target(cross.position, my_pos_2d, id, robots))
    return DEFENDER;

  if (!is_duplicate_target(cross_spec.position, my_pos_2d, id, robots))
    return SPECULATIVE_DEFENDER;

  if (attack_spec.exists and
      !is_duplicate_target(attack_spec.position, my_pos_2d, id, robots))
    return SPECULATIVE_ATTACKER;

  return DEFAULT;
}

double MyStrategy::calc_jump_speed(
    const Vec3D &my_position,
    const Vec3D &ball_position,
    const int &id) {
  double dist_to_ball = (my_position - ball_position).len();

  Role role = roles[id];

  TargetJump ball_intercept = calc_jump_intercept(
     projected_jump_paths[id],
     projected_ball_path,
     my_position);
  TargetJump ball_spec_intercept = calc_jump_intercept(
     projected_jump_paths[id],
     projected_ball_spec_path,
     my_position);

  if (role == SPECULATIVE_DEFENDER and ball_spec_intercept.exists)
    return rules.ROBOT_MAX_JUMP_SPEED;

  if (not ball_intercept.exists)
    return 0.0;

  if (role == AGGRESSIVE_DEFENDER or
      (role == DEFENDER and ball_intercept.robot_pos.z <= CRITICAL_BORDER))
    return rules.ROBOT_MAX_JUMP_SPEED;

  double acceptable_dist = rules.BALL_RADIUS + 6*rules.ROBOT_MAX_RADIUS;

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
  for (int i = 0; i < std::min(int(robot_path.size()), int(ball_path.size())); ++i)
    if ((ball_path[i] - robot_path[i]).len() <= rules.BALL_RADIUS + rules.ROBOT_RADIUS) {
      if (ball_path[i].z > my_position.z and
          ball_path[i].z > robot_path[i].z + 0.5 and
          ball_path[i].y > robot_path[i].y) {
        return {true, ball_path[i], robot_path[i]};
      } else
        return {false, Vec3D(), Vec3D()};
    }
  return {false, Vec3D(), Vec3D()};
}

bool MyStrategy::is_duplicate_target(
    const Vec2D &target_position,
    const Vec2D &my_position,
    const int &id,
    const std::vector<Robot> &robots) {
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
  renderer.clear();

  // draw borders
  renderer.draw_border(DEFENSE_BORDER);
  renderer.draw_border(CRITICAL_BORDER);

  // predicted paths of the ball
  renderer.draw_ball_path(projected_ball_path, 2, RED, 0.25);
  renderer.draw_ball_path(projected_ball_spec_path, 2, VIOLET, 0.1);

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

  // predicted jump paths of the robots
  for (int id = 1; id < int(projected_jump_paths.size()); ++id) {
    Vec3D start_pos = projected_jump_paths[id][0];
    if (start_pos.y > rules.ROBOT_RADIUS)
      renderer.draw_line(start_pos, {start_pos.x, start_pos.z, 20}, 10, YELLOW, 0.5);
    else
      renderer.draw_line(start_pos, {start_pos.x, start_pos.z, 20}, 10, TEAL, 0.5);
    for (int i = 1; i < int(projected_jump_paths[id].size()); ++i) {
      Vec3D position = projected_jump_paths[id][i];
      renderer.draw_sphere(position, 1, YELLOW, 0.5);
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
      case SPECULATIVE_ATTACKER:
        renderer.draw_sphere(hover,                0.5, VIOLET, 1.0);
        renderer.draw_sphere(target_positions[id], 1.0, VIOLET, 0.5);
        break;
      case SPECULATIVE_DEFENDER:
        renderer.draw_sphere(hover,                0.5, LIGHT_BLUE, 1.0);
        renderer.draw_sphere(target_positions[id], 1.0, LIGHT_BLUE, 0.5);
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
