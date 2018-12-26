#include "MyStrategy.h"

using namespace model;

MyStrategy::MyStrategy() { }

const double SIMULATION_DURATION = 5;
const double SIMULATION_PRECISION = 1/60.0;

void MyStrategy::act(const Robot& me, const Rules& rules, const Game& game, Action& action) {
  if (game.current_tick == 0) {
    std::cout<<"START!\n";

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
      // roles.push_back(GUARD);
    }
  }

  bool is_start_of_round = Vec2D(game.ball.x, game.ball.z).len() < rules.BALL_RADIUS;

  Vec3D my_position = {me.x, me.z, me.y};
  Vec3D ball_position = {game.ball.x, game.ball.z, game.ball.y};

  // Recalculate the predictions of the ball's path on new tick
  if (prev_tick != game.current_tick) {
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

  double dist_to_ball = (my_position - ball_position).len();

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

  // The defender automatically becomes an attacker when the ball reaches
  // 1/3 of the arena from our goal
  if (game.ball.z <= -rules.arena.depth / 6.0)
    is_attacker = true;

  // At the start of the round, 2 robots must attack the ball
  if (is_start_of_round and game.robots.size() == 4) {
    if (!is_attacker and dist_to_ball < rules.BALL_RADIUS + 4*rules.ROBOT_MAX_RADIUS)
      jump = true;
    is_attacker = true;
  }

  if (is_attacker) {
    // Attacker strategy
    for (int i = 1; i < int(predicted_ball_positions.size()); ++i) {
      if (goal_scored(predicted_ball_positions[i].z))
        break;

      double t = i * SIMULATION_PRECISION;

      Vec2D target_pos(predicted_ball_positions[i].x, predicted_ball_positions[i].z);
      // TODO: Change this if the target is near the goal
      target_pos.z -= 1.5*rules.ROBOT_RADIUS;

      // If ball will not leave arena boundary
      // (collision with the arena would happen, but we are not considering it),
      // and the ball will be closer to opponent's net than the robot,
      if (target_pos.z > me.z and predicted_ball_positions[i].y <= 4*rules.ROBOT_RADIUS) {
        // Compute the speed robot needs to run with
        // To be at ball's location at the same time as the ball
        Vec2D delta_pos = target_pos - Vec2D(me.x, me.z);

        double need_speed = delta_pos.len() / t;
        // If the speed is in acceptable range
        if (0.5 * rules.ROBOT_MAX_GROUND_SPEED < need_speed and need_speed < rules.ROBOT_MAX_GROUND_SPEED) {
          // If a robot already is headed there and it is closer, then discard this
          // However, if it is just the start of the round, don't check this
          if (!is_start_of_round and
              is_duplicate_target(target_pos, delta_pos, me.id, game.robots))
            break;
            // should I use `continue` instead?

          Vec2D target_velocity = delta_pos.normalize() * need_speed;
          set_action(
            action,
            me.id,
            Vec3D(target_pos.x, target_pos.z, 0.0),
            Vec3D(target_velocity.x, target_velocity.z, 0.0),
            (jump ? rules.ROBOT_MAX_JUMP_SPEED : 0.0),
            false
          );
          return;
        }
      }
    }
  }

  // Defender's strategy (or attacker's who did not find good moment):
  // Standing in the middle of out net
  //Vec2D target_pos(0.0, -(rules.arena.depth/2.0) + rules.arena.bottom_radius));
  Vec2D target_pos(0.0, -(rules.arena.depth/2.0) - rules.BALL_RADIUS);

  // Find time and place where ball crosses the net line
  // If this place is inside the net
  // Go defend there
  for (int i = 1; i < int(predicted_ball_positions.size()); ++i) {
    if (predicted_ball_positions[i].z <= -rules.arena.depth/2.0) {
      target_pos.x = predicted_ball_positions[i].x;
      break;
    }
  }

  Vec2D target_velocity = (target_pos - Vec2D(me.x, me.z)) * rules.ROBOT_MAX_GROUND_SPEED;

  set_action(
    action,
    me.id,
    Vec3D(target_pos.x, target_pos.z, 0.0),
    Vec3D(target_velocity.x, target_velocity.z, 0.0),
    (jump ? rules.ROBOT_MAX_JUMP_SPEED : 0.0),
    false
  );

  prev_tick = game.current_tick;
}

bool MyStrategy::goal_scored(double z) {
  return std::fabs(z) > rules.arena.depth/2.0 + rules.BALL_RADIUS;
}

bool MyStrategy::is_duplicate_target(const Vec2D &target_pos, const Vec2D &delta_pos, int id, const std::vector<Robot> &robots) {
  for (Robot robot : robots) {
    if (!robot.is_teammate or robot.id == id)
      continue;

    Vec2D target_pos_other = Vec2D(
      target_positions[robot.id].x,
      target_positions[robot.id].z
    );
    Vec2D delta_pos_other = target_pos_other - Vec2D(robot.x, robot.z);

    if ((target_pos - target_pos_other).len() < rules.BALL_RADIUS and
        delta_pos_other.len() < delta_pos.len()) {
      return true;
    }
  }
  return false;
}

void MyStrategy::set_action(model::Action &action,
                            int id,
                            const Vec3D &target_position,
                            const Vec3D &target_velocity,
                            double jump_speed,
                            bool use_nitro) {
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

  /*  b -- a
   *  |    |
   *  c -- d
   */
  Vec3D corner_A = Vec3D( 25, -11.7, 19);
  Vec3D corner_B = Vec3D(-25, -11.7, 19);
  Vec3D corner_C = Vec3D(-25, -11.7,  1);
  Vec3D corner_D = Vec3D( 25, -11.7,  1);
  // defense border (cross)
  res +=       draw_line_util(corner_A, corner_C, 5, 0, 1, 0, 0.5);
  res += "," + draw_line_util(corner_B, corner_D, 5, 0, 1, 0, 0.5);
  // defense border (box)
  res += "," + draw_line_util(corner_A, corner_B, 5, 0, 1, 0, 0.5);
  res += "," + draw_line_util(corner_B, corner_C, 5, 0, 1, 0, 0.5);
  res += "," + draw_line_util(corner_C, corner_D, 5, 0, 1, 0, 0.5);
  res += "," + draw_line_util(corner_D, corner_A, 5, 0, 1, 0, 0.5);

  // predicted path of the ball
  for (int i = 0; i < int(predicted_ball_positions.size()); ++i) {
    if (goal_scored(predicted_ball_positions[i].z)) {
      res += "," + draw_sphere_util(predicted_ball_positions[i], 1, 1, 0, 0, 1.0);
      break;
    }
    res += "," + draw_sphere_util(predicted_ball_positions[i], 1, 1, 0, 0, 0.25);
  }
  // predicted path of the ball_col
  for (int i = 0; i < int(predicted_ball_col_positions.size()); ++i) {
    if (goal_scored(predicted_ball_col_positions[i].z)) {
      res += "," + draw_sphere_util(predicted_ball_col_positions[i], 1, 0.5, 0, 0.5, 1);
      break;
    }
    res += "," + draw_sphere_util(predicted_ball_col_positions[i], 1, 0.5, 0, 0.5, 0.1);
  }

  // predicted paths of the robots
  for (int j = 0; j < int(predicted_robot_positions[0].size()); ++j) {
    for (int i = 1; i < int(predicted_robot_positions.size()); ++i) {
      Vec3D prev_pos = predicted_robot_positions[i-1][j];
      Vec3D position = predicted_robot_positions[i][j];
      res += "," + draw_line_util(prev_pos, position, 10, 0, 0, 1, 0.5);
    }
  }

  // target positions of the robots
  for (int id : ally_ids)
    res += "," + draw_sphere_util(target_positions[id], 1.0, 0.0, 1.0, 1.0, 0.5);

  res += "]";
  return res;
}

std::string MyStrategy::draw_sphere_util(const Vec3D &pos, double radius, double r, double g, double b, double a) {
  std::string pos_str = "\"x\":" + std::to_string(pos.x) + "," +
                        "\"y\":" + std::to_string(pos.y) + "," +
                        "\"z\":" + std::to_string(pos.z);
  std::string radius_str = "\"radius\":" + std::to_string(radius);
  std::string color_str = "\"r\":" + std::to_string(r) + "," +
                          "\"g\":" + std::to_string(g) + "," +
                          "\"b\":" + std::to_string(b) + "," +
                          "\"a\":" + std::to_string(a);
  return "{\"Sphere\":{" + pos_str + "," + radius_str + "," + color_str + "}}";
}

std::string MyStrategy::draw_line_util(const Vec3D &p1, const Vec3D &p2, double width, double r, double g, double b, double a) {
  std::string p1_str = "\"x1\":" + std::to_string(p1.x) + "," +
                       "\"y1\":" + std::to_string(p1.y) + "," +
                       "\"z1\":" + std::to_string(p1.z);
  std::string p2_str = "\"x2\":" + std::to_string(p2.x) + "," +
                       "\"y2\":" + std::to_string(p2.y) + "," +
                       "\"z2\":" + std::to_string(p2.z);
  std::string width_str = "\"width\":" + std::to_string(width);
  std::string color_str = "\"r\":" + std::to_string(r) + "," +
                          "\"g\":" + std::to_string(g) + "," +
                          "\"b\":" + std::to_string(b) + "," +
                          "\"a\":" + std::to_string(a);
  return "{\"Line\": {" + p1_str + "," + p2_str + "," + width_str + "," + color_str + "}}";
}
