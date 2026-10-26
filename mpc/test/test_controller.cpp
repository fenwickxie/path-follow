#include "path_follow_mpc/controller.hpp"

#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

using namespace path_follow_mpc;

namespace
{
void require(bool condition, const std::string & message)
{
  if (!condition) {
    throw std::runtime_error(message);
  }
}

template<typename Function>
void require_invalid(Function function, const std::string & message)
{
  bool caught = false;
  try {
    function();
  } catch (const std::invalid_argument &) {
    caught = true;
  }
  require(caught, message);
}

void check_control(const Config & c, const Control & u, const Control & previous)
{
  constexpr double epsilon = 1e-8;
  require(std::abs(u.vx) <= c.max_vx + epsilon, "vx limit");
  require(std::abs(lateral_velocity(u, c)) <= c.max_vy + epsilon, "physical vy limit");
  require(std::abs(yaw_rate(u, c)) <= c.max_yaw_rate + epsilon, "yaw rate limit");
  require(std::abs(u.vx - previous.vx) <= c.max_acceleration * c.dt + epsilon, "vx slew");
  require(std::abs(u.vy - previous.vy) <= c.max_lateral_acceleration * c.dt + epsilon,
    "vy slew");
  const bool four = c.chassis == Chassis::FourWheelSteering;
  const bool ack = c.chassis == Chassis::Ackermann || four;
  require(std::abs(u.turn) <= (ack ? c.max_steering : c.max_yaw_rate) + epsilon, "turn limit");
  require(std::abs(u.turn - previous.turn) <=
    (ack ? c.max_steering_rate : c.max_yaw_acceleration) * c.dt + epsilon, "turn slew");
  if (c.chassis != Chassis::Omnidirectional) {
    require(u.vx >= -epsilon && u.vy == 0.0, "nonholonomic forward-only constraint");
  }
  if (four) {
    require(std::abs(u.rear_turn) <= c.max_rear_steering + epsilon, "rear steering limit");
    require(std::abs(u.rear_turn - previous.rear_turn) <=
      c.max_rear_steering_rate * c.dt + epsilon, "rear steering slew");
  } else {
    require(u.rear_turn == 0.0, "unused rear steering must be zero");
  }
}

void test_validation()
{
  Config c;
  c.horizon = 0;
  require_invalid([&c]() {Controller controller(c);}, "reject invalid horizon");
  c = Config{};
  c.dt = std::numeric_limits<double>::quiet_NaN();
  require_invalid([&c]() {Controller controller(c);}, "reject NaN dt");
  c = Config{};
  c.max_steering = 1.6;
  require_invalid([&c]() {Controller controller(c);}, "reject singular steering");
  c = Config{};
  c.chassis = Chassis::FourWheelSteering;
  c.front_axle_distance = 0.0;
  require_invalid([&c]() {Controller controller(c);}, "reject invalid front axle distance");
  c = Config{};
  c.max_rear_steering = 1.6;
  require_invalid([&c]() {Controller controller(c);}, "reject singular rear steering");
  c = Config{};
  c.max_rear_steering_rate = -0.1;
  require_invalid([&c]() {Controller controller(c);}, "reject negative rear steering rate");
  Controller controller(Config{});
  require(!controller.compute({}, {}).valid, "reject missing path");
  require_invalid([&controller]() {controller.set_path({});}, "reject empty core path");
  controller.set_path({{0, 0, 0}, {0, 0, 0}, {1, 0, 0}});
  require_invalid([&controller]() {
    controller.set_path({{std::numeric_limits<double>::infinity(), 0, 0}});
  }, "reject infinite path");
  require(controller.has_path(), "invalid replacement must preserve core path");
  require(!controller.compute({0, 0, std::numeric_limits<double>::quiet_NaN()}, {}).valid,
    "reject invalid robot state");
  require(!controller.compute({}, {2.0, 0, 0}).valid, "reject out-of-bounds previous control");
  require(!controller.compute({}, {0, 0, 0, 0.1}).valid, "reject rear command in Ackermann");
  require(!controller.compute({}, {0, 0, 0, std::numeric_limits<double>::quiet_NaN()}).valid,
    "reject non-finite rear command");
  controller.clear_path();
  require(!controller.has_path(), "clear path");
  controller.set_path({{1, 2, 3.141592653589793}});
  const auto result = controller.compute({1, 2, -3.141592653589793}, {});
  require(result.valid && result.goal_reached, "one point goal and wrapped yaw");
  require(result.command.vx == 0 && result.command.turn == 0, "goal command must stop");
  controller.set_path({{0, 0, 0}, {0.01, 0, 0}, {1.7, 0, 0}, {2.0, 0, 0}});
  const auto irregular = controller.compute({0, 0.1, 0}, {});
  require(irregular.valid && irregular.prediction.size() == Config{}.horizon + 1,
    "nonuniform path");
  controller.set_path({{0, 0, 0}, {1, 0, 0}, {1, 1, 1.5707963267948966},
    {0, 1, 3.141592653589793}, {0, 0, -1.5707963267948966}});
  const auto loop = controller.compute({}, {});
  require(loop.valid && !loop.goal_reached, "closed loop must not finish at its starting point");
}

void test_kinematics()
{
  Config c;
  c.dt = 0.2;
  c.chassis = Chassis::Omnidirectional;
  const auto strafe = integrate({0, 0, 1.5707963267948966}, {0, 1, 0}, c);
  require(std::abs(strafe.x + 0.2) < 1e-10 && std::abs(strafe.y) < 1e-10,
    "omni velocity must be body-frame");
  c.chassis = Chassis::Ackermann;
  const Control u{0.6, 0.0, 0.3};
  const auto arc = integrate({}, u, c);
  const double w = yaw_rate(u, c);
  require(std::abs(arc.x - u.vx / w * std::sin(w * c.dt)) < 1e-10, "exact arc x");
  require(std::abs(arc.y - u.vx / w * (1.0 - std::cos(w * c.dt))) < 1e-10, "exact arc y");

  c.chassis = Chassis::FourWheelSteering;
  const Control crab{0.6, 0, 0.2, 0.2};
  const auto translated = integrate({}, crab, c);
  require(std::abs(yaw_rate(crab, c)) < 1e-12, "same-phase steering has zero yaw rate");
  require(std::abs(translated.x - 0.6 * c.dt) < 1e-12 &&
    std::abs(translated.y - 0.6 * std::tan(0.2) * c.dt) < 1e-12, "crab translation");
  const Control opposite{0.6, 0, 0.2, -0.2};
  require(std::abs(lateral_velocity(opposite, c)) < 1e-12, "symmetric opposite steering vy");
  require(std::abs(yaw_rate(opposite, c) - 1.2 * std::tan(0.2) / 0.65) < 1e-12,
    "symmetric opposite steering yaw rate");
  require(yaw_rate({0, 0, 0.2, -0.2}, c) == 0.0, "four wheel cannot rotate at rest");
  // 改变参考点位置后，两轴的无侧滑条件仍应同时成立。
  c.front_axle_distance = 0.2;
  c.rear_axle_distance = 0.45;
  const Control asymmetric{0.7, 0, 0.3, -0.1};
  const double vy = lateral_velocity(asymmetric, c);
  const double omega = yaw_rate(asymmetric, c);
  require(std::abs(vy + c.front_axle_distance * omega -
    asymmetric.vx * std::tan(asymmetric.turn)) < 1e-12, "front axle no-slip equation");
  require(std::abs(vy - c.rear_axle_distance * omega -
    asymmetric.vx * std::tan(asymmetric.rear_turn)) < 1e-12, "rear axle no-slip equation");
  c.rear_axle_distance = 0.325;
  c.front_axle_distance = 0.325;
  const Control front_only{0.6, 0, 0.2, 0.0};
  require(std::abs(yaw_rate(front_only, c) - 0.6 * std::tan(0.2) / 0.65) < 1e-12,
    "front-only four wheel yaw rate");
  require(std::abs(lateral_velocity(front_only, c) -
    c.rear_axle_distance * yaw_rate(front_only, c)) < 1e-12, "front-only reference offset");
}

void simulate(Chassis chassis, bool curve)
{
  Config c;
  c.chassis = chassis;
  Controller controller(c);
  std::vector<Pose> path;
  for (int i = 0; i <= 60; ++i) {
    const double s = static_cast<double>(i) * 0.06;
    if (curve) {
      const double radius = 3.0;
      path.push_back({radius * std::sin(s / radius),
        radius * (1.0 - std::cos(s / radius)), s / radius});
    } else {
      path.push_back({s, 0, 0});
    }
  }
  controller.set_path(path);
  Pose state{0, 0.35, 0.08};
  Control previous;
  bool reached = false;
  bool opposite_steering_seen = false;
  double squared_error = 0.0;
  double max_ms = 0.0;
  int samples = 0;
  for (int step = 0; step < 220; ++step) {
    const auto start = std::chrono::steady_clock::now();
    const auto result = controller.compute(state, previous);
    max_ms = std::max(max_ms, std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count());
    require(result.valid, "closed-loop solver failure: " + result.message);
    if (result.goal_reached) {
      reached = true;
      break;
    }
    require(result.prediction.size() == c.horizon + 1, "prediction shape");
    require(std::isfinite(result.cost), "finite cost");
    check_control(c, result.command, previous);
    opposite_steering_seen = opposite_steering_seen ||
      (result.command.turn > 0.01 && result.command.rear_turn < -0.01);
    state = integrate(state, result.command, c);
    previous = result.command;
    if (step > 20) {
      const double error = curve ? std::abs(std::hypot(state.x, state.y - 3.0) - 3.0) :
        std::abs(state.y);
      squared_error += error * error;
      ++samples;
    }
  }
  const double error = std::hypot(state.x - path.back().x, state.y - path.back().y);
  const double yaw_error = std::abs(normalize_angle(state.yaw - path.back().yaw));
  const double rms = std::sqrt(squared_error / std::max(1, samples));
  std::cout << "chassis=" << static_cast<int>(chassis) << " curve=" << curve <<
    " reached=" << reached << " final_error=" << error << " yaw_error=" << yaw_error <<
    " rms=" << rms << " max_solve_ms=" << max_ms << '\n';
  require(reached, "closed-loop goal must be reached");
  require(error <= c.goal_position_tolerance, "terminal position tolerance");
  require(yaw_error <= c.goal_yaw_tolerance, "terminal yaw tolerance");
  require(rms < 0.12, "cross-track RMS must be less than 0.12m");
  if (chassis == Chassis::FourWheelSteering && curve) {
    require(opposite_steering_seen, "four wheel curved path must use counter-phase steering");
  }
}

void test_omni_strafe()
{
  Config c;
  c.chassis = Chassis::Omnidirectional;
  Controller controller(c);
  controller.set_path({{0, 0, 0}, {0, 2, 0}});
  Pose state;
  Control previous;
  bool reached = false;
  for (int i = 0; i < 150; ++i) {
    const auto result = controller.compute(state, previous);
    require(result.valid, "strafe solution valid");
    if (result.goal_reached) {
      reached = true;
      break;
    }
    check_control(c, result.command, previous);
    require(std::abs(result.command.vx) < 0.02, "pure strafe vx");
    require(std::abs(state.yaw) < 0.05, "pure strafe holds heading");
    state = integrate(state, result.command, c);
    previous = result.command;
  }
  require(reached, "omni strafe goal");
}

void test_rotation()
{
  Config c;
  c.chassis = Chassis::Differential;
  Controller controller(c);
  controller.set_path({{0, 0, 1.0}});
  Pose state;
  Control previous;
  bool reached = false;
  for (int i = 0; i < 100; ++i) {
    const auto result = controller.compute(state, previous);
    require(result.valid, "rotation solution");
    if (result.goal_reached) {
      reached = true;
      break;
    }
    check_control(c, result.command, previous);
    state = integrate(state, result.command, c);
    previous = result.command;
  }
  require(reached && std::hypot(state.x, state.y) < 0.01, "differential rotates in place");
}

void test_four_wheel_crab(bool restrictive_limits)
{
  Config c;
  c.chassis = Chassis::FourWheelSteering;
  if (restrictive_limits) {
    c.max_yaw_rate = 0.3;
    c.max_vy = 0.18;
    c.max_rear_steering = 0.2;
    c.max_rear_steering_rate = 0.25;
    c.front_axle_distance = 0.25;
    c.rear_axle_distance = 0.4;
  }
  Controller controller(c);
  // 路径方向与车身朝向相差 atan(0.08)，仅需斜向蟹行，不是全向底盘的 90 度横移。
  controller.set_path({{0, 0, 0}, {3, 0.24, 0}});
  Pose state;
  Control previous;
  bool reached = false;
  bool same_phase_seen = false;
  for (int i = 0; i < 220; ++i) {
    const auto result = controller.compute(state, previous);
    require(result.valid, "four wheel crab valid");
    if (result.goal_reached) {
      reached = true;
      require(result.command.rear_turn == 0.0 && result.command.vx == 0.0, "crab goal stop");
      break;
    }
    check_control(c, result.command, previous);
    same_phase_seen = same_phase_seen ||
      (result.command.turn > 0.01 && result.command.rear_turn > 0.01);
    state = integrate(state, result.command, c);
    previous = result.command;
    require(std::abs(state.yaw) < 0.08, "crab must hold body heading");
  }
  require(reached && same_phase_seen, "crab goal and same-phase steering");
}

void test_four_wheel_limit_corners()
{
  Config c;
  c.chassis = Chassis::FourWheelSteering;
  c.max_vy = 0.12;
  c.max_yaw_rate = 0.25;
  c.max_rear_steering = 0.06;
  c.max_rear_steering_rate = 0.1;
  const double tangent_limit = std::min(c.max_vy / c.max_vx,
    c.max_yaw_rate * (c.front_axle_distance + c.rear_axle_distance) / (2.0 * c.max_vx));
  const double front_limit = std::min(c.max_steering, std::atan(tangent_limit));
  const double rear_limit = std::min(c.max_rear_steering, std::atan(tangent_limit));
  // 从约束矩形的四个角出发，验证高速时同时改变两轴仍符合物理速度及独立速率限制。
  for (const double front_sign : {-1.0, 1.0}) {
    for (const double rear_sign : {-1.0, 1.0}) {
      Controller controller(c);
      controller.set_path({{0, 0, 0}, {4, 0.2, 0}});
      Control previous{c.max_vx, 0.0, front_sign * front_limit, rear_sign * rear_limit};
      const auto result = controller.compute({}, previous);
      require(result.valid, "four wheel bound corner previous accepted");
      check_control(c, result.command, previous);
      const auto predicted = integrate({}, result.command, c);
      require(std::abs(predicted.x - result.prediction[1].x) < 1e-12 &&
        std::abs(predicted.y - result.prediction[1].y) < 1e-12,
        "four wheel prediction matches first issued command");
      require(!controller.compute({}, {0, 0, front_limit + 0.01, 0}).valid,
        "effective front bound enforced on previous control");
      require(!controller.compute({}, {0, 0, 0, rear_limit + 0.01}).valid,
        "effective rear bound enforced on previous control");
    }
  }
}
}  // namespace

int main()
{
  try {
    test_validation();
    test_kinematics();
    for (const auto chassis :
      {Chassis::Ackermann, Chassis::Differential, Chassis::Omnidirectional,
        Chassis::FourWheelSteering})
    {
      simulate(chassis, false);
      simulate(chassis, true);
    }
    test_omni_strafe();
    test_rotation();
    test_four_wheel_crab(false);
    test_four_wheel_crab(true);
    test_four_wheel_limit_corners();
    std::cout << "All core tests passed\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    return 1;
  }
}
