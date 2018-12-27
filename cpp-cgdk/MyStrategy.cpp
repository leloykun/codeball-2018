#include "MyStrategy.h"

using namespace model;

MyStrategy::MyStrategy() { }

void MyStrategy::act(const Robot& me, const Rules& rules, const Game& game, Action& action) {
  if (game.current_tick == 0 and not this->initialized)
    this->init_strategy(rules, game);

  Vec2D my_position_2d = {me.x, me.z};
  Vec3D my_position_3d = {my_position_2d, me.y};
  Vec2D ball_position_2d = {game.ball.x, game.ball.z};
  Vec3D ball_position_3d = {ball_position_2d, game.ball.y};

  this->robot_positions[me.id] = my_position_3d;

  // Recalculate the predictions of the ball's path on new tick
  if (this->prev_tick != game.current_tick) {
    this->is_start_of_round = ball_position_2d.len() < rules.BALL_RADIUS;

    if (game.current_tick % 100 == 0)
      std::cout<<game.current_tick<<"\n";

    this->run_simulation(game);

    this->prev_tick = game.current_tick;
  }

  Target attack = calc_intercept_spot(projected_ball_path, my_position_2d);
  Target attack_spec = calc_intercept_spot(
    projected_ball_spec_path,
    my_position_2d,
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
    attack_spec,
    cross,
    cross_spec);


  Vec2D target_position;
  Vec2D target_velocity;

  switch (roles[me.id]) {
    case ATTACKER:
    case INIT_JUMPER:
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
    calc_jump_speed(my_position_3d, ball_position_3d, roles[me.id]),
    false
  );
}

void MyStrategy::init_strategy(
    const model::Rules &rules,
    const model::Game &game) {
  std::cout<<"START!\n";

  this->DEFENSE_BORDER = -rules.arena.depth / 6.0;
  this->CRITICAL_BORDER = -(rules.arena.depth/2.0 - rules.arena.top_radius);
  this->rules = rules;

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
    this->roles.push_back(DEFAULT);
  }

  this->initialized = true;
}

void MyStrategy::run_simulation(const model::Game &game) {
  Vec3D ball_position_3d = {game.ball.x, game.ball.z, game.ball.y};

  // initiate projections
  projected_ball_path = {ball_position_3d};
  projected_ball_spec_path = {ball_position_3d};
  projected_robot_paths = std::vector<std::vector<Vec3D>>(game.robots.size()+1);
  for (Robot robot : game.robots) {
    projected_robot_paths[robot.id].push_back({
      robot.x, robot.z, robot.y
    });
  }

  Simulation sim(game.ball,
                 game.robots,
                 target_velocities,
                 jump_speeds,
                 rules,
                 SIMULATION_PRECISION);
  for(int i = 1; i <= SIMULATION_DURATION/SIMULATION_PRECISION; ++i) {
    sim.update();
    projected_ball_path.push_back(sim.ball.position);
    projected_ball_spec_path.push_back(sim.ball_spec.position);
    for (Entity robot : sim.robots)
      projected_robot_paths[robot.id].push_back(robot.position);
  }
}

Target MyStrategy::calc_intercept_spot(
    const std::vector<Vec3D> &ball_path,
    const Vec2D &my_position,
    const bool &is_speculative,
    const std::vector<Vec3D> &avoid_path) {

  Vec2D target_position;
  Vec2D target_velocity;

  for (int i = 1; i < int(ball_path.size()); ++i) {
    if (goal_scored(ball_path[i].z))
      break;
    if (is_speculative and (ball_path[i] - avoid_path[i]).len() < rules.BALL_RADIUS)
      continue;

    double t = i * SIMULATION_PRECISION;

    target_position.x = ball_path[i].x;
    target_position.z = ball_path[i].z - 1.5 * rules.ROBOT_RADIUS;

    // If ball will not leave arena boundary
    // (collision with the arena would happen, but we are not considering it),
    // and the ball will be closer to opponent's net than the robot,
    if (target_position.z > my_position.z and
        ball_path[i].y <= 4*rules.ROBOT_RADIUS) {
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
    const std::vector<Vec3D> &ball_path,
    const Vec2D &my_position) {

  Vec2D target_position(0.0, -(rules.arena.depth/2.0 + rules.BALL_RADIUS));
  Vec2D target_velocity = (target_position - my_position) *
                          rules.ROBOT_MAX_GROUND_SPEED;

  for (int i = 1; i < int(ball_path.size()); ++i) {
    if (goal_scored(ball_path[i].z))
      break;

    double t = i * SIMULATION_PRECISION;

    if (ball_path[i].z <= -rules.arena.depth/2.0) {
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
  Vec2D target_position(ball_position.x, -rules.arena.depth/2.0);
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
      attack.exists and
      attack.position.z <= DEFENSE_BORDER)
    role = AGGRESSIVE_DEFENDER;

  if (role == DEFENDER and is_start_of_round and robots.size() == 4)
    role = INIT_JUMPER;

  if (attack.exists and (role == ATTACKER or
                         role == AGGRESSIVE_DEFENDER or
                         role == INIT_JUMPER)) {
    if (is_start_of_round or
        !is_duplicate_target(attack.position, my_pos_2d, id, robots))
      return role;
  }

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
    const Role &role) {
  double dist_to_ball = (my_position - ball_position).len();

  double jump_dist_factor = 2;
  if (role == INIT_JUMPER or
      role == AGGRESSIVE_DEFENDER or
      role == DEFENDER or
      role == SPECULATIVE_DEFENDER or
      my_position.z < CRITICAL_BORDER)
    jump_dist_factor = 4;

  double acceptable_dist = rules.BALL_RADIUS + jump_dist_factor*rules.ROBOT_MAX_RADIUS;

  if (my_position.z < ball_position.z and dist_to_ball < acceptable_dist) {
    return rules.ROBOT_MAX_JUMP_SPEED;
  }
  return 0.0;
}

bool MyStrategy::goal_scored(double z) {
  return std::fabs(z) > rules.arena.depth/2.0 + rules.BALL_RADIUS;
}

bool MyStrategy::is_duplicate_target(const Vec2D &target_position,
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
  }
  return false;
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
  std::string res = "[";

  std::string defense_border = draw_border_util(DEFENSE_BORDER);
  std::string critical_border = draw_border_util(CRITICAL_BORDER);
  res += defense_border + "," + critical_border;

  // predicted path of the ball
  for (int i = 0; i < int(projected_ball_path.size()); ++i) {
    if (goal_scored(projected_ball_path[i].z)) {
      res += "," + draw_sphere_util(projected_ball_path[i], 1.5, RED, 1.0);
      break;
    }
    res += "," + draw_sphere_util(projected_ball_path[i], 1, RED, 0.25);
  }
  // predicted path of the ball_spec
  for (int i = 0; i < int(projected_ball_spec_path.size()); ++i) {
    if (goal_scored(projected_ball_spec_path[i].z)) {
      res += "," + draw_sphere_util(projected_ball_spec_path[i], 1.5, VIOLET, 1);
      break;
    }
    res += "," + draw_sphere_util(projected_ball_spec_path[i], 1, VIOLET, 0.1);
  }

  // predicted paths of the robots
  for (int id = 1; id < int(projected_robot_paths.size()); ++id) {
    for (int i = 1; i < int(projected_robot_paths[id].size()); ++i) {
      int step_size = 6;
      if (i % step_size == 0 and 2*i <= SIMULATION_DURATION/SIMULATION_PRECISION) {
        Vec3D prev_pos = projected_robot_paths[id][i-step_size];
        Vec3D position = projected_robot_paths[id][i];
        res += "," + draw_line_util(prev_pos, position, 10, TEAL, 0.5);
      }
    }
  }

  // roles of the robots
  for (int id : ally_ids) {
    Vec3D hover = robot_positions[id];
    hover.y += 1.5*rules.ROBOT_RADIUS;
    switch(roles[id]) {
      case ATTACKER:
        res += "," + draw_sphere_util(hover,                0.5, RED, 1.0);
        res += "," + draw_sphere_util(target_positions[id], 1.0, RED, 0.5);
        break;
      case INIT_JUMPER:
        res += "," + draw_sphere_util(hover,                0.5, YELLOW, 1.0);
        res += "," + draw_sphere_util(target_positions[id], 1.0, YELLOW, 0.5);
        break;
      case DEFENDER:
        res += "," + draw_sphere_util(hover,                0.5, BLUE, 1.0);
        res += "," + draw_sphere_util(target_positions[id], 1.0, BLUE, 0.5);
        break;
      case AGGRESSIVE_DEFENDER:
        res += "," + draw_sphere_util(hover,                0.5, LIGHT_RED, 1.0);
        res += "," + draw_sphere_util(target_positions[id], 1.0, LIGHT_RED, 0.5);
        break;
      case SPECULATIVE_DEFENDER:
        res += "," + draw_sphere_util(hover,                0.5, LIGHT_BLUE, 1.0);
        res += "," + draw_sphere_util(target_positions[id], 1.0, LIGHT_BLUE, 0.5);
        break;
      case SPECULATIVE_ATTACKER:
        res += "," + draw_sphere_util(hover,                0.5, VIOLET, 1.0);
        res += "," + draw_sphere_util(target_positions[id], 1.0, VIOLET, 0.5);
        break;
      case DEFAULT:
        res += "," + draw_sphere_util(hover,                0.5, WHITE, 1.0);
        res += "," + draw_sphere_util(target_positions[id], 1.0, WHITE, 0.5);
        break;
      default:
        res += "," + draw_sphere_util(hover,                0.5, BLACK, 1.0);
        res += "," + draw_sphere_util(target_positions[id], 1.0, BLACK, 0.5);
    };
  }

  res += "]";
  return res;
}

std::string MyStrategy::draw_border_util(const double &border_z) {
  std::string res = "";
  /*  b -- a
   *  |    |
   *  c -- d
   */
  Vec3D corner_A = Vec3D( 22, border_z, 17);
  Vec3D corner_B = Vec3D(-22, border_z, 17);
  Vec3D corner_C = Vec3D(-22, border_z,  1);
  Vec3D corner_D = Vec3D( 22, border_z,  1);
  // border (cross)
  res +=       draw_line_util(corner_A, corner_C, 5, GREEN, 0.5);
  res += "," + draw_line_util(corner_B, corner_D, 5, GREEN, 0.5);
  // border (box)
  res += "," + draw_line_util(corner_A, corner_B, 5, GREEN, 0.5);
  res += "," + draw_line_util(corner_B, corner_C, 5, GREEN, 0.5);
  res += "," + draw_line_util(corner_C, corner_D, 5, GREEN, 0.5);
  res += "," + draw_line_util(corner_D, corner_A, 5, GREEN, 0.5);
  return res;
}

std::string MyStrategy::draw_sphere_util(const Vec3D &pos,
                                         const double &radius,
                                         const Color &color,
                                         const double &alpha) {
  std::string pos_str = "\"x\":" + std::to_string(pos.x) + "," +
                        "\"y\":" + std::to_string(pos.y) + "," +
                        "\"z\":" + std::to_string(pos.z);
  std::string radius_str = "\"radius\":" + std::to_string(radius);
  std::string color_str = "\"r\":" + std::to_string(color.r) + "," +
                          "\"g\":" + std::to_string(color.g) + "," +
                          "\"b\":" + std::to_string(color.b) + "," +
                          "\"a\":" + std::to_string(alpha);
  return "{\"Sphere\":{" + pos_str + "," + radius_str + "," + color_str + "}}";
}

std::string MyStrategy::draw_line_util(const Vec3D &p1,
                                       const Vec3D &p2,
                                       const double &width,
                                       const Color &color,
                                       const double &alpha) {
  std::string p1_str = "\"x1\":" + std::to_string(p1.x) + "," +
                       "\"y1\":" + std::to_string(p1.y) + "," +
                       "\"z1\":" + std::to_string(p1.z);
  std::string p2_str = "\"x2\":" + std::to_string(p2.x) + "," +
                       "\"y2\":" + std::to_string(p2.y) + "," +
                       "\"z2\":" + std::to_string(p2.z);
  std::string width_str = "\"width\":" + std::to_string(width);
  std::string color_str = "\"r\":" + std::to_string(color.r) + "," +
                          "\"g\":" + std::to_string(color.g) + "," +
                          "\"b\":" + std::to_string(color.b) + "," +
                          "\"a\":" + std::to_string(alpha);
  return "{\"Line\": {" + p1_str + "," + p2_str + "," + width_str + "," + color_str + "}}";
}
