#include "MyStrategy.h"
#include "Simulation.h"

using namespace model;

MyStrategy::MyStrategy() { }

const double SIMULATION_DURATION = 5;
const double SIMULATION_PRECISION = 1/60.0;

void MyStrategy::act(const Robot& me, const Rules& rules, const Game& game, Action& action) {
  std::cout<<"CUR TICK "<<game.current_tick<<"\n";

  Vec3D my_position = {me.x, me.z, me.y};
  Vec3D ball_position = {game.ball.x, game.ball.z, game.ball.y};

  // Recalculate the predictions of the ball's path on new tick
  if (prev_tick != game.current_tick) {
    predicted_ball_positions = {ball_position};
    predicted_robot_positions = {{}};
    for (Robot robot : game.robots) {
      predicted_robot_positions.back().push_back({
        robot.x, robot.z, robot.y
      });
    }
    target_positions = {};

    Simulation sim(game.ball, game.robots, rules, SIMULATION_PRECISION);
    for(int i = 1; i <= SIMULATION_DURATION/SIMULATION_PRECISION; ++i) {
      sim.update();
      predicted_ball_positions.push_back(sim.ball.position);
      if (i % 6 == 0) {
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
    action.target_velocity_x = 0.0;
    action.target_velocity_y = -rules.MAX_ENTITY_SPEED;
    action.target_velocity_z = 0.0;
    action.jump_speed = 0.0;
    action.use_nitro = true;
    return;
  }

  double dist_to_ball = (my_position - ball_position).len();

  // Lets jump if we would hit the ball, and
  // we are on the same side of the ball as out net, so
  // the ball would go into opponent's side of the arena
  bool jump = (dist_to_ball < rules.BALL_RADIUS + 2*rules.ROBOT_MAX_RADIUS) and (me.z < game.ball.z);

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
  if (me.z < game.ball.z and game.ball.z <= -rules.arena.depth / 6.0)
    is_attacker = true;

  // At the start of the round, 2 robots must attack the ball
  if (game.robots.size() == 4 and Vec2D(game.ball.x, game.ball.z).len() < rules.BALL_RADIUS) {
    if (!is_attacker and (dist_to_ball < rules.BALL_RADIUS + 4*rules.ROBOT_MAX_RADIUS))
      jump = true;
    is_attacker = true;
  }

  if (is_attacker) {
    // Attacker strategy
    for (int i = 1; i < int(predicted_ball_positions.size()); ++i) {
      double t = i * SIMULATION_PRECISION;

      Vec2D target_pos(predicted_ball_positions[i].x, predicted_ball_positions[i].z);
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
          Vec2D target_velocity = delta_pos.normalize() * need_speed;
          action.target_velocity_x = target_velocity.x;
          action.target_velocity_y = 0.0;
          action.target_velocity_z = target_velocity.z;
          action.jump_speed = (jump ? rules.ROBOT_MAX_JUMP_SPEED : 0.0);
          action.use_nitro = false;

          target_positions.push_back(target_pos);
          return;
        }
      }
    }
  }

  // Defender's strategy (or attacker's who did not find good moment):
  // Standing in the middle of out net
  Vec2D target_pos(0.0, -(rules.arena.depth/2.0) + rules.arena.bottom_radius);
  // And, if the ball is rolling towards
  if (game.ball.velocity_z < -EPS) {
    // Find time and place where ball crosses the net line
    // If this place is inside the net
    // Go defend there
    for (int i = 1; i < int(predicted_ball_positions.size()); ++i) {
      if (predicted_ball_positions[i].z <= -rules.arena.depth/2.0) {
        target_pos.x = predicted_ball_positions[i].x;
        break;
      }
    }
  }

  target_positions.push_back(target_pos);
  Vec2D target_velocity = (target_pos - Vec2D(me.x, me.z)) * rules.ROBOT_MAX_GROUND_SPEED;

  action.target_velocity_x = target_velocity.x;
  action.target_velocity_y = 0.0;
  action.target_velocity_z = target_velocity.z;
  action.jump_speed = (jump ? rules.ROBOT_MAX_JUMP_SPEED : 0.0);
  action.use_nitro = false;

  prev_tick = game.current_tick;

  // std::cout<<"Finished tick "<<game.current_tick<<"\n";
}

std::string MyStrategy::custom_rendering() {
  return convert_positions_to_string(predicted_ball_positions);
}

std::string MyStrategy::draw_sphere_util(const Vec3D &pos, double radius, double r, double g, double b) {
  std::string pos_str = "\"x\":" + std::to_string(pos.x) + "," +
                        "\"y\":" + std::to_string(pos.y) + "," +
                        "\"z\":" + std::to_string(pos.z);
  std::string radius_str = "\"radius\":" + std::to_string(radius);
  std::string color_str = "\"r\":" + std::to_string(r) + "," +
                          "\"g\":" + std::to_string(g) + "," +
                          "\"b\":" + std::to_string(b) + "," +
                          "\"a\":" + std::to_string(0.5);
  return "{\"Sphere\":{" + pos_str + "," + radius_str + "," + color_str + "}}";
}

std::string MyStrategy::draw_sphere_util(const Vec2D &pos) {
  return "{\"Sphere\":{\"x\":"+std::to_string(pos.x)+",\"y\":0.0,\"z\":"+std::to_string(pos.z)+",\"radius\":1.0,\"r\":0.0,\"g\":0.0,\"b\": 1.0,\"a\":0.5}}";
}

std::string MyStrategy::convert_positions_to_string(const std::vector<Vec3D> &positions) {
  std::string res = "[";
  // predicted path of the ball
  for (int i = 0; i < int(positions.size()); ++i) {
    if (i)  res += ",";
    res += draw_sphere_util(positions[i], 1, 1, 0, 0);
  }
  // predicted paths of the robots
  for (int j = 0; j < int(predicted_robot_positions[0].size()); ++j) {
    for (int i = 0; i < int(predicted_robot_positions.size()); ++i) {
      Vec3D position = predicted_robot_positions[i][j];
      res += "," + draw_sphere_util(position, 0.5, 0, 1, 0);
    }
  }
  // defense border
  res += ",{\"Line\": {\"x1\": -25.0,\"y1\": 0.0,\"z1\": -11.7,\"x2\": 25.0,\"y2\": 20.0,\"z2\": -11.7,\"width\": 1.5,\"r\": 0.0,\"g\": 0.0,\"b\": 1.0,\"a\": 1.0}}";
  res += ",{\"Line\": {\"x1\": 25.0,\"y1\": 0.0,\"z1\": -11.7,\"x2\": -25.0,\"y2\": 20.0,\"z2\": -11.7,\"width\": 1.5,\"r\": 0.0,\"g\": 0.0,\"b\": 1.0,\"a\": 1.0}}";
  res += ",{\"Line\": {\"x1\": 25.0,\"y1\": 20.0,\"z1\": -11.7,\"x2\": -25.0,\"y2\": 20.0,\"z2\": -11.7,\"width\": 1.5,\"r\": 0.0,\"g\": 0.0,\"b\": 1.0,\"a\": 1.0}}";
  res += ",{\"Line\": {\"x1\": 25.0,\"y1\": 0.0,\"z1\": -11.7,\"x2\": -25.0,\"y2\": 0.0,\"z2\": -11.7,\"width\": 1.5,\"r\": 0.0,\"g\": 0.0,\"b\": 1.0,\"a\": 1.0}}";
  // target position
  for (auto target_pos : target_positions)
    res += "," + draw_sphere_util(target_pos);
  res += "]";
  return res;
}
