#include "MyStrategy.h"

using namespace model;

MyStrategy::MyStrategy() { }

const double SIMULATION_DURATION = 5;
const double SIMULATION_PRECISION = 1/60.0;

void MyStrategy::act(const Robot& me, const Rules& rules, const Game& game, Action& action) {
  if (game.current_tick == 0) {
    std::cout<<"START!\n";

    DEFENSE_BORDER = -rules.arena.depth / 6.0;
    CRITICAL_BORDER = -(rules.arena.depth/2.0 - rules.arena.top_radius);
    this->rules = rules;

    // get IDs of allies
    for (Robot robot : game.robots)
      if (robot.is_teammate)
        ally_ids.push_back(robot.id);

    // fix target_positions
    for (int i = 0; i < int(game.robots.size()); ++i) {
      target_positions.push_back(Vec3D());
      target_velocities.push_back(Vec3D());
      jump_speeds.push_back(0.0);
      robot_positions.push_back(Vec3D());
      roles.push_back(DEFAULT);
    }
  }


  Vec2D my_position_2d = {me.x, me.z};
  Vec3D my_position_3d = {my_position_2d, me.y};
  Vec3D ball_position = {game.ball.x, game.ball.z, game.ball.y};

  robot_positions[me.id] = my_position_3d;

  // Recalculate the predictions of the ball's path on new tick
  if (prev_tick != game.current_tick) {
    is_start_of_round = Vec2D(game.ball.x, game.ball.z).len() < rules.BALL_RADIUS;

    if (game.current_tick % 100 == 0)
      std::cout<<game.current_tick<<"\n";

    predicted_ball_positions = {ball_position};
    predicted_ball_col_positions = {ball_position};
    predicted_robot_positions = {{}};
    for (Robot robot : game.robots) {
      predicted_robot_positions.back().push_back({
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
      predicted_ball_positions.push_back(sim.ball.position);
      predicted_ball_col_positions.push_back(sim.ball_col.position);
      if (i % 6 == 0 and 2*i <= SIMULATION_DURATION/SIMULATION_PRECISION) {
        predicted_robot_positions.push_back({});
        for (Entity robot : sim.robots)
          predicted_robot_positions.back().push_back(robot.position);
      }
    }
    prev_tick = game.current_tick;
  }

  // The strategy only plays on the ground
  // So, if we are not touching the ground, use nitro
  // to go back as soon as possible
  if (!me.touch) {
    set_action(
      action,
      me.id,
      Vec3D(me.x, me.z, 0.0),
      Vec3D(0.0, 0.0, -rules.MAX_ENTITY_SPEED),
      0.0,
      true
    );
    // return
  }


  double dist_to_ball = (my_position_3d - ball_position).len();

  // Lets jump if we would hit the ball, and
  // we are on the same side of the ball as out net, so
  // the ball would go into opponent's side of the arena
  bool jump = ((dist_to_ball < rules.BALL_RADIUS + 2*rules.ROBOT_MAX_RADIUS) and
               (me.z < game.ball.z));


  // Since there are multiple robots in out team lets determine out role - attacker or defender

  // This robot is automatically an attacker if it is alone
  bool is_attacker = (game.robots.size() == 2);

  // We will be attacker if there is friendly robot closer
  // to out net than current one.
  for (const model::Robot &robot : game.robots)
    if (robot.is_teammate and robot.id != me.id and robot.z < me.z)
      is_attacker = true;

  roles[me.id] = (is_attacker ? ATTACKER : DEFENDER);

  // The defender automatically becomes an attacker when the ball reaches
  // 1/3 of the arena from our goal
  if (!is_attacker and game.ball.z <= DEFENSE_BORDER) {
    is_attacker = true;
    roles[me.id] = AGGRESSIVE_DEFENDER;
  }

  // At the start of the round, 2 robots must attack the ball
  if (is_start_of_round and game.robots.size() == 4) {
    if (!is_attacker and dist_to_ball < rules.BALL_RADIUS + 4*rules.ROBOT_MAX_RADIUS)
      jump = true;
    is_attacker = true;
    roles[me.id] = ATTACKER;
  }


  if (is_attacker) {
    if (me.z < CRITICAL_BORDER)
      jump = ((dist_to_ball < rules.BALL_RADIUS + 4*rules.ROBOT_MAX_RADIUS) and
                   (me.z < game.ball.z));

    // Attacker strategy
    Vec2D attack_position;
    Vec2D attack_velocity;
    if (find_intercept_spot(predicted_ball_positions,
                            my_position_2d,
                            attack_position,
                            attack_velocity)) {
      set_action(
        action,
        me.id,
        Vec3D(attack_position, 0.0),
        Vec3D(attack_velocity, 0.0),
        (jump ? rules.ROBOT_MAX_JUMP_SPEED : 0.0),
        false
      );
      // If a robot already is headed there and it is closer, then discard this
      // However, if it is just the start of the round, don't check this
      if ((is_start_of_round or
           !is_duplicate_target(attack_position, my_position_2d, me.id, game.robots)) and
          !(roles[me.id] == AGGRESSIVE_DEFENDER and attack_position.z > DEFENSE_BORDER)) {
        return;
      }
    }
  }

  // Defender's strategy (or attacker's who did not find good moment):
  roles[me.id] = DEFENDER;
  // Standing in the middle of out net
  Vec2D defend_position(0.0, -(rules.arena.depth/2.0) - rules.BALL_RADIUS);

  // Find time and place where ball crosses the net line
  // If this place is inside the net
  // Go defend there
  bool ball_might_cross = false;
  double t = EPS;
  for (int i = 1; i < int(predicted_ball_positions.size()); ++i) {
    if (goal_scored(predicted_ball_positions[i].z))
      break;
    t = i * SIMULATION_PRECISION;
    if (predicted_ball_positions[i].z <= -rules.arena.depth/2.0) {
      ball_might_cross = true;
      defend_position.x = predicted_ball_positions[i].x;
      break;
    }
  }
  if (!ball_might_cross) {
    t = EPS;
    for (int i = 1; i < int(predicted_ball_col_positions.size()); ++i) {
      if (goal_scored(predicted_ball_col_positions[i].z))
        break;
      t = i * SIMULATION_PRECISION;
      if (predicted_ball_col_positions[i].z <= -rules.arena.depth/2.0) {
        ball_might_cross = true;
        defend_position.x = predicted_ball_col_positions[i].x;
        roles[me.id] = SPECULATIVE_DEFENDER;
        break;
      }
    }
  }

  // go to the target position ASAP
  Vec2D defend_velocity = (defend_position - my_position_2d) *
                          rules.ROBOT_MAX_GROUND_SPEED;

  // if the defender is already inside the net,
  // then just slowly go to the target position
  Vec2D delta_pos = defend_position - my_position_2d;
  double need_speed = delta_pos.len() / t;

  Vec2D needed_velocity = delta_pos.normalize() * need_speed;
  if (ball_might_cross and me.z < CRITICAL_BORDER)
    defend_velocity = needed_velocity;

  // TODO: FIX THIS LATER
  jump = ((dist_to_ball < rules.BALL_RADIUS + 4*rules.ROBOT_MAX_RADIUS) and
               (me.z < game.ball.z));

  set_action(
    action,
    me.id,
    Vec3D(defend_position, 0.0),
    Vec3D(defend_velocity, 0.0),
    (jump ? rules.ROBOT_MAX_JUMP_SPEED : 0.0),
    false
  );

  if (!is_duplicate_target(defend_position, my_position_2d, me.id, game.robots))
    return;


  // Speculative strategy
  roles[me.id] = SPECULATIVE_ATTACKER;
  Vec2D specul_position;
  Vec2D specul_velocity;
  if (find_intercept_spot(predicted_ball_col_positions,
                          my_position_2d,
                          specul_position,
                          specul_velocity,
                          true,
                          predicted_ball_positions)) {
    set_action(
      action,
      me.id,
      Vec3D(specul_position, 0.0),
      Vec3D(specul_velocity, 0.0),
      (jump ? rules.ROBOT_MAX_JUMP_SPEED : 0.0),
      false
    );
    if (!is_duplicate_target(specul_position, my_position_2d, me.id, game.robots))
      return;
  }

  // Default
  roles[me.id] = DEFAULT;
  // TODODODODODOD: KNOCK OFF OPPONENTS
  Vec2D default_position(ball_position.x, -rules.arena.depth/2.0);
  Vec2D default_velocity = (default_position - my_position_2d) *
                            rules.ROBOT_MAX_GROUND_SPEED;
  set_action(
    action,
    me.id,
    Vec3D(default_position, 0.0),
    Vec3D(default_velocity, 0.0),
    (jump ? rules.ROBOT_MAX_JUMP_SPEED : 0.0),
    false
  );
}

bool MyStrategy::find_intercept_spot(const std::vector<Vec3D> &predicted_ball_path,
                                     const Vec2D &my_position,
                                     Vec2D &target_position,
                                     Vec2D &target_velocity,
                                     const bool &is_speculative,
                                     const std::vector<Vec3D> &avoid_path) {
  for (int i = 1; i < int(predicted_ball_path.size()); ++i) {
    if (goal_scored(predicted_ball_path[i].z))
      break;
    if (is_speculative and (predicted_ball_path[i] - avoid_path[i]).len() < rules.BALL_RADIUS)
      continue;

    double t = i * SIMULATION_PRECISION;

    target_position.x = predicted_ball_path[i].x;
    target_position.z = predicted_ball_path[i].z - 1.5 * rules.ROBOT_RADIUS;

    // If ball will not leave arena boundary
    // (collision with the arena would happen, but we are not considering it),
    // and the ball will be closer to opponent's net than the robot,
    if (target_position.z > my_position.z and
        predicted_ball_path[i].y <= 4*rules.ROBOT_RADIUS) {
      // Compute the speed robot needs to run with
      // To be at ball's location at the same time as the ball
      Vec2D delta_pos = target_position - my_position;

      double need_speed = delta_pos.len() / t;
      // If the speed is in acceptable range
      if (0.5 * rules.ROBOT_MAX_GROUND_SPEED < need_speed and
          need_speed < rules.ROBOT_MAX_GROUND_SPEED) {
        target_velocity = delta_pos.normalize() * need_speed;
        return true;
      }
    }
  }
  return false;
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

void MyStrategy::set_action(model::Action &action,
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
  for (int i = 0; i < int(predicted_ball_positions.size()); ++i) {
    if (goal_scored(predicted_ball_positions[i].z)) {
      res += "," + draw_sphere_util(predicted_ball_positions[i], 1, RED, 1.0);
      break;
    }
    res += "," + draw_sphere_util(predicted_ball_positions[i], 1, RED, 0.25);
  }
  // predicted path of the ball_col
  for (int i = 0; i < int(predicted_ball_col_positions.size()); ++i) {
    if (goal_scored(predicted_ball_col_positions[i].z)) {
      res += "," + draw_sphere_util(predicted_ball_col_positions[i], 1, VIOLET, 1);
      break;
    }
    res += "," + draw_sphere_util(predicted_ball_col_positions[i], 1, VIOLET, 0.1);
  }

  // predicted paths of the robots
  for (int j = 0; j < int(predicted_robot_positions[0].size()); ++j) {
    for (int i = 1; i < int(predicted_robot_positions.size()); ++i) {
      Vec3D prev_pos = predicted_robot_positions[i-1][j];
      Vec3D position = predicted_robot_positions[i][j];
      res += "," + draw_line_util(prev_pos, position, 10, BLUE, 0.5);
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
    }
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
