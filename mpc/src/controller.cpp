#include "path_follow_mpc/controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace path_follow_mpc
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kEpsilon = 1e-9;

double square(double value) {return value * value;}
bool finite(const Pose & p)
{
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.yaw);
}
bool finite(const Control & u)
{
  return std::isfinite(u.vx) && std::isfinite(u.vy) && std::isfinite(u.turn) &&
         std::isfinite(u.rear_turn);
}
double component(const Control & u, std::size_t i)
{
  return i == 0 ? u.vx : (i == 1 ? u.vy : (i == 2 ? u.turn : u.rear_turn));
}
void set_component(Control & u, std::size_t i, double value)
{
  if (i == 0) {
    u.vx = value;
  } else if (i == 1) {
    u.vy = value;
  } else if (i == 2) {
    u.turn = value;
  } else {
    u.rear_turn = value;
  }
}

// 使用固定而非随 vx 收缩的转角边界，避免加速时破坏相邻步转角速率约束。
// 四轮转向中为前后轴各预留一半角速度预算；各轴同时满足横向速度预算。
// 因而任何允许的两轴组合都满足 |w|<=max_yaw_rate、|vy|<=max_vy。
std::array<double, 2> steering_limits(const Config & c)
{
  if (c.chassis == Chassis::FourWheelSteering) {
    const double tangent_limit = std::min(
      c.max_yaw_rate * (c.front_axle_distance + c.rear_axle_distance) / (2.0 * c.max_vx),
      c.max_vy / c.max_vx);
    return {std::min(c.max_steering, std::atan(tangent_limit)),
      std::min(c.max_rear_steering, std::atan(tangent_limit))};
  }
  return {c.chassis == Chassis::Ackermann ? std::min(c.max_steering,
    std::atan(c.max_yaw_rate * c.wheelbase / c.max_vx)) : c.max_yaw_rate, 0.0};
}
}  // namespace

double normalize_angle(double angle)
{
  return std::remainder(angle, 2.0 * kPi);
}

double yaw_rate(const Control & control, const Config & config)
{
  if (config.chassis == Chassis::FourWheelSteering) {
    return control.vx * (std::tan(control.turn) - std::tan(control.rear_turn)) /
           (config.front_axle_distance + config.rear_axle_distance);
  }
  return config.chassis == Chassis::Ackermann ?
         control.vx * std::tan(control.turn) / config.wheelbase : control.turn;
}

double lateral_velocity(const Control & control, const Config & config)
{
  if (config.chassis == Chassis::FourWheelSteering) {
    // 无侧滑约束：vy + lf*w = vx*tan(df)，vy - lr*w = vx*tan(dr)。
    // 联立得参考点横向速度；同相转角时 w=0，实现斜向蟹行。
    return control.vx *
           (config.rear_axle_distance * std::tan(control.turn) +
           config.front_axle_distance * std::tan(control.rear_turn)) /
           (config.front_axle_distance + config.rear_axle_distance);
  }
  return config.chassis == Chassis::Omnidirectional ? control.vy : 0.0;
}

Pose integrate(const Pose & pose, const Control & control, const Config & config)
{
  const double w = yaw_rate(control, config);
  const double vy = lateral_velocity(control, config);
  const double half_turn = 0.5 * w * config.dt;
  const double scale = std::abs(half_turn) < kEpsilon ? 1.0 :
    std::sin(half_turn) / half_turn;
  const double heading = pose.yaw + half_turn;
  return {
    pose.x + config.dt * scale * (control.vx * std::cos(heading) - vy * std::sin(heading)),
    pose.y + config.dt * scale * (control.vx * std::sin(heading) + vy * std::cos(heading)),
    normalize_angle(pose.yaw + w * config.dt)};
}

void validate_config(const Config & c)
{
  if (c.chassis != Chassis::Ackermann && c.chassis != Chassis::Differential &&
    c.chassis != Chassis::Omnidirectional && c.chassis != Chassis::FourWheelSteering)
  {
    throw std::invalid_argument("Unknown chassis model");
  }
  if (c.horizon < 2 || c.horizon > 100 || c.max_iterations < 1 || c.max_iterations > 1000) {
    throw std::invalid_argument("horizon must be in [2,100], max_iterations in [1,1000]");
  }
  const std::array<double, 23> positive{
    c.dt, c.wheelbase, c.target_speed, c.max_vx, c.max_vy, c.max_yaw_rate,
    c.max_steering, c.max_acceleration, c.max_lateral_acceleration,
    c.max_yaw_acceleration, c.max_steering_rate, c.position_weight,
    c.heading_weight, c.terminal_weight, c.gradient_tolerance,
    c.goal_position_tolerance, c.goal_yaw_tolerance, c.progress_search_distance,
    c.velocity_weight, c.front_axle_distance, c.rear_axle_distance,
    c.max_rear_steering, c.max_rear_steering_rate};
  for (double value : positive) {
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument("MPC scales, limits, tolerances and tracking weights must be positive");
    }
  }
  if (!std::isfinite(c.effort_weight) || c.effort_weight < 0.0 ||
    !std::isfinite(c.smoothness_weight) || c.smoothness_weight < 0.0 ||
    c.max_steering >= kPi / 2.0 || c.max_rear_steering >= kPi / 2.0 ||
    !std::isfinite(c.front_axle_distance + c.rear_axle_distance) ||
    c.target_speed > c.max_vx)
  {
    throw std::invalid_argument("Invalid weights, steering limit or target speed");
  }
}

Controller::Controller(Config config) : config_(std::move(config))
{
  validate_config(config_);
}

void Controller::set_path(const std::vector<Pose> & path)
{
  if (path.empty()) {
    throw std::invalid_argument("Path must contain at least one pose");
  }
  std::vector<Pose> cleaned;
  std::vector<double> stations;
  for (const auto & p : path) {
    if (!finite(p)) {
      throw std::invalid_argument("Path contains non-finite pose");
    }
    if (cleaned.empty()) {
      cleaned.push_back({p.x, p.y, normalize_angle(p.yaw)});
      stations.push_back(0.0);
      continue;
    }
    const double distance = std::hypot(p.x - cleaned.back().x, p.y - cleaned.back().y);
    if (distance < 1e-6) {
      cleaned.back().yaw = normalize_angle(p.yaw);
    } else {
      cleaned.push_back({p.x, p.y, normalize_angle(p.yaw)});
      stations.push_back(stations.back() + distance);
    }
  }
  path_ = std::move(cleaned);
  stations_ = std::move(stations);
  warm_start_.clear();
  progress_ = 0.0;
  progress_initialized_ = false;
}

void Controller::clear_path()
{
  path_.clear();
  stations_.clear();
  warm_start_.clear();
  progress_ = 0.0;
  progress_initialized_ = false;
}

bool Controller::has_path() const {return !path_.empty();}

double Controller::update_progress(const Pose & pose)
{
  if (path_.size() == 1) {
    return 0.0;
  }
  double best_distance = std::numeric_limits<double>::infinity();
  double best_station = progress_;
  // 向每条候选线段做正交投影，比较投影点距离；不是简单选最近离散点。
  // 限制搜索前向窗口可减少交叉路径上“跳到后续分支”的风险。
  for (std::size_t i = 1; i < path_.size(); ++i) {
    if (progress_initialized_ &&
      (stations_[i] < progress_ ||
      stations_[i - 1] > progress_ + config_.progress_search_distance))
    {
      continue;
    }
    const auto & a = path_[i - 1];
    const auto & b = path_[i];
    const double length = stations_[i] - stations_[i - 1];
    const double lower = progress_initialized_ ?
      std::clamp((progress_ - stations_[i - 1]) / length, 0.0, 1.0) : 0.0;
    const double upper = progress_initialized_ ?
      std::clamp((progress_ + config_.progress_search_distance - stations_[i - 1]) / length,
      lower, 1.0) : 1.0;
    const double t = std::clamp(
      ((pose.x - a.x) * (b.x - a.x) + (pose.y - a.y) * (b.y - a.y)) /
      square(length), lower, upper);
    const double distance = square(pose.x - a.x - t * (b.x - a.x)) +
      square(pose.y - a.y - t * (b.y - a.y));
    if (distance < best_distance) {
      best_distance = distance;
      best_station = stations_[i - 1] + t * length;
    }
  }
  progress_ = std::max(progress_, best_station);
  progress_initialized_ = true;
  return progress_;
}

Pose Controller::sample(double station) const
{
  if (path_.size() == 1 || station >= stations_.back()) {
    return path_.back();
  }
  const auto upper = std::upper_bound(stations_.begin(), stations_.end(), station);
  const std::size_t i = static_cast<std::size_t>(std::distance(stations_.begin(), upper));
  const auto & a = path_[i - 1];
  const auto & b = path_[i];
  const double t = (station - stations_[i - 1]) / (stations_[i] - stations_[i - 1]);
  // 四轮转向/全向允许车身朝向不同于路径切线，必须保留输入 yaw 才能表达蟹行。
  const double yaw = (config_.chassis == Chassis::Omnidirectional ||
    config_.chassis == Chassis::FourWheelSteering) ?
    normalize_angle(a.yaw + t * normalize_angle(b.yaw - a.yaw)) :
    std::atan2(b.y - a.y, b.x - a.x);
  return {a.x + t * (b.x - a.x), a.y + t * (b.y - a.y), yaw};
}

std::vector<Controller::Reference> Controller::references(double progress) const
{
  std::vector<Reference> refs;
  refs.reserve(config_.horizon);
  double station = progress;
  for (std::size_t i = 0; i < config_.horizon; ++i) {
    const Pose before_step = sample(station);
    const double remaining = std::max(0.0, stations_.back() - station);
    const double speed = std::min(config_.target_speed,
      std::sqrt(2.0 * config_.max_acceleration * remaining));
    station = std::min(stations_.back(), station + speed * config_.dt);
    Pose p = sample(station);
    Control u{speed, 0.0, 0.0};
    if ((config_.chassis == Chassis::Omnidirectional ||
      config_.chassis == Chassis::FourWheelSteering) && remaining > kEpsilon)
    {
      const Pose before = sample(std::max(0.0, station - 0.01));
      const Pose after = sample(std::min(stations_.back(), station + 0.01));
      const double direction = std::atan2(after.y - before.y, after.x - before.x) - p.yaw;
      u.vx = speed * std::cos(direction);
      const double vy = speed * std::sin(direction);
      if (config_.chassis == Chassis::FourWheelSteering) {
        const double w = normalize_angle(p.yaw - before_step.yaw) / config_.dt;
        // 从期望本体速度反解两轴前馈转角；可行性仍由后续 project 保证。
        u.turn = std::atan2(vy + config_.front_axle_distance * w, u.vx);
        u.rear_turn = std::atan2(vy - config_.rear_axle_distance * w, u.vx);
      } else {
        u.vy = vy;
      }
    }
    refs.push_back({p, u});
  }
  return refs;
}

Control Controller::constrain(Control u, const Control & previous) const
{
  const auto bounded = [this](double value, double before, double low, double high, double rate) {
      before = std::clamp(before, low, high);
      return std::clamp(value, std::max(low, before - rate * config_.dt),
        std::min(high, before + rate * config_.dt));
    };
  const bool omni = config_.chassis == Chassis::Omnidirectional;
  u.vx = bounded(u.vx, previous.vx, omni ? -config_.max_vx : 0.0,
    config_.max_vx, config_.max_acceleration);
  u.vy = omni ? bounded(u.vy, previous.vy, -config_.max_vy, config_.max_vy,
    config_.max_lateral_acceleration) : 0.0;
  const bool four = config_.chassis == Chassis::FourWheelSteering;
  const bool steering = config_.chassis == Chassis::Ackermann || four;
  const auto limits = steering_limits(config_);
  u.turn = bounded(u.turn, previous.turn, -limits[0], limits[0],
    steering ? config_.max_steering_rate : config_.max_yaw_acceleration);
  u.rear_turn = four ? bounded(u.rear_turn, previous.rear_turn, -limits[1], limits[1],
    config_.max_rear_steering_rate) : 0.0;
  return u;
}

std::vector<Control> Controller::project(
  const std::vector<Control> & controls, const Control & previous) const
{
  auto result = controls;
  Control before = previous;
  for (auto & u : result) {
    // 后一步的速率边界依赖前一步，必须按时间顺序处理整条序列。
    u = constrain(u, before);
    before = u;
  }
  return result;
}

double Controller::evaluate(
  const Pose & pose, const Control & previous, const std::vector<Control> & controls,
  const std::vector<Reference> & refs, std::vector<Pose> * prediction) const
{
  Pose state = pose;
  Control before = previous;
  double cost = 0.0;
  if (prediction) {
    prediction->clear();
    prediction->push_back(state);
  }
  for (std::size_t i = 0; i < controls.size(); ++i) {
    const auto & u = controls[i];
    const auto & ref = refs[i];
    state = integrate(state, u, config_);
    const double terminal = i + 1 == controls.size() ? config_.terminal_weight : 1.0;
    cost += terminal * (config_.position_weight *
      (square(state.x - ref.pose.x) + square(state.y - ref.pose.y)) +
      config_.heading_weight * square(normalize_angle(state.yaw - ref.pose.yaw)));
    const double vy = lateral_velocity(u, config_);
    cost += config_.velocity_weight *
      (square(u.vx - ref.control.vx) +
      square(vy - lateral_velocity(ref.control, config_)));
    cost += config_.effort_weight * (square(vy) + square(u.turn) + square(u.rear_turn));
    cost += config_.smoothness_weight *
      (square(u.vx - before.vx) + square(u.vy - before.vy) + square(u.turn - before.turn) +
      square(u.rear_turn - before.rear_turn));
    before = u;
    if (prediction) {
      prediction->push_back(state);
    }
  }
  return cost;
}

Result Controller::compute(const Pose & pose, const Control & previous)
{
  Result result;
  if (!finite(pose) || !finite(previous)) {
    result.message = "Non-finite robot state or previous control";
    return result;
  }
  const bool four = config_.chassis == Chassis::FourWheelSteering;
  const bool omni = config_.chassis == Chassis::Omnidirectional;
  const auto limits = steering_limits(config_);
  if (std::abs(previous.vx) > config_.max_vx + kEpsilon ||
    (!omni && (previous.vx < 0.0 || previous.vy != 0.0)) ||
    std::abs(previous.vy) > config_.max_vy + kEpsilon ||
    std::abs(previous.turn) > limits[0] + kEpsilon ||
    std::abs(previous.rear_turn) > limits[1] + kEpsilon ||
    (!four && previous.rear_turn != 0.0))
  {
    result.message = "Previous control is outside configured model limits";
    return result;
  }
  if (!has_path()) {
    result.message = "No active path";
    return result;
  }
  const double progress = update_progress(pose);
  const auto & goal = path_.back();
  if (stations_.back() - progress <= config_.goal_position_tolerance &&
    std::hypot(pose.x - goal.x, pose.y - goal.y) <= config_.goal_position_tolerance &&
    std::abs(normalize_angle(pose.yaw - goal.yaw)) <= config_.goal_yaw_tolerance)
  {
    result.valid = true;
    result.goal_reached = true;
    result.converged = true;
    result.message = "Goal reached";
    result.prediction.push_back(pose);
    warm_start_.clear();
    return result;
  }
  const auto refs = references(progress);
  std::vector<Control> seed;
  seed.reserve(config_.horizon);
  for (const auto & ref : refs) {
    seed.push_back(ref.control);
  }
  auto controls = project(seed, previous);
  double cost = evaluate(pose, previous, controls, refs);
  if (warm_start_.size() == config_.horizon) {
    // 上次第 0 步已执行，把第 1 步移到最前，再与新参考种子比较，避免盲信旧轨迹。
    auto warm = warm_start_;
    std::rotate(warm.begin(), warm.begin() + 1, warm.end());
    warm.back() = warm[warm.size() - 2];
    warm = project(warm, previous);
    const double warm_cost = evaluate(pose, previous, warm, refs);
    if (warm_cost < cost) {
      controls = std::move(warm);
      cost = warm_cost;
    }
  }
  constexpr double epsilon = 1e-5;
  for (std::size_t iteration = 0; iteration < config_.max_iterations; ++iteration) {
    // 中央差分：(J(u+eps)-J(u-eps))/(2*eps)。仅对当前模型的有效控制维度求导。
    // 扰动用于估计光滑代价，不在此限幅；真正试探命令在线搜索阶段统一投影。
    std::vector<Control> gradient(config_.horizon);
    for (std::size_t i = 0; i < config_.horizon; ++i) {
      for (std::size_t j = 0; j < 4; ++j) {
        if ((j == 1 && !omni) || (j == 3 && !four)) {
          continue;
        }
        auto perturbed = controls;
        const double value = component(controls[i], j);
        set_component(perturbed[i], j, value + epsilon);
        const double plus = evaluate(pose, previous, perturbed, refs);
        set_component(perturbed[i], j, value - epsilon);
        const double minus = evaluate(pose, previous, perturbed, refs);
        set_component(gradient[i], j, (plus - minus) / (2.0 * epsilon));
      }
    }
    bool accepted = false;
    double step = 0.1;
    double change = 0.0;
    // 步长过大会越过下降区域；每次减半，并且只接受严格降低总代价的可行序列。
    for (int search = 0; search < 18; ++search) {
      auto candidate = controls;
      for (std::size_t i = 0; i < config_.horizon; ++i) {
        candidate[i].vx -= step * gradient[i].vx;
        candidate[i].vy -= step * gradient[i].vy;
        candidate[i].turn -= step * gradient[i].turn;
        candidate[i].rear_turn -= step * gradient[i].rear_turn;
      }
      candidate = project(candidate, previous);
      const double candidate_cost = evaluate(pose, previous, candidate, refs);
      if (std::isfinite(candidate_cost) && candidate_cost < cost) {
        change = cost - candidate_cost;
        controls = std::move(candidate);
        cost = candidate_cost;
        accepted = true;
        break;
      }
      step *= 0.5;
    }
    result.iterations = iteration + 1;
    if (!accepted || change < config_.gradient_tolerance) {
      // Sequential rate projection is feasible but not an orthogonal projection.
      // A stalled search is not proof of constrained optimality.
      result.converged = accepted && change < config_.gradient_tolerance;
      break;
    }
  }
  result.cost = evaluate(pose, previous, controls, refs, &result.prediction);
  if (!std::isfinite(result.cost) ||
    !std::all_of(controls.begin(), controls.end(), [](const Control & u) {return finite(u);}) ||
    !std::all_of(result.prediction.begin(), result.prediction.end(),
    [](const Pose & p) {return finite(p);}))
  {
    result.message = "Non-finite optimization result";
    result.prediction.clear();
    warm_start_.clear();
    return result;
  }
  warm_start_ = controls;
  result.valid = true;
  // 滚动时域控制只执行第一项，下一周期用新测量重新求解，而非开环执行整个窗口。
  result.command = controls.front();
  result.message = result.converged ? "Cost tolerance reached" : "Feasible bounded MPC solution";
  return result;
}

}  // namespace path_follow_mpc
