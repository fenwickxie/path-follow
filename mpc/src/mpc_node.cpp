#include "path_follow_mpc/controller.hpp"

#include <chrono>
#include <csignal>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "path_follow_mpc/msg/four_wheel_steering_command.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

namespace path_follow_mpc
{
namespace
{
using SteadyClock = std::chrono::steady_clock;

Pose planar_pose(const geometry_msgs::msg::Pose & message)
{
  const auto & p = message.position;
  const auto & q = message.orientation;
  if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
    !std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w))
  {
    throw std::invalid_argument("Pose contains non-finite position or quaternion");
  }
  tf2::Quaternion quaternion(q.x, q.y, q.z, q.w);
  if (quaternion.length2() < 1e-12) {
    throw std::invalid_argument("Pose quaternion has zero norm");
  }
  quaternion.normalize();
  double roll, pitch, yaw;
  tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
  return {p.x, p.y, yaw};
}

geometry_msgs::msg::Pose to_message(const Pose & p)
{
  geometry_msgs::msg::Pose result;
  result.position.x = p.x;
  result.position.y = p.y;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, p.yaw);
  result.orientation = tf2::toMsg(q);
  return result;
}
}  // namespace

class MpcNode : public rclcpp::Node
{
public:
  explicit MpcNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("mpc_tracker", options)
  {
    Config config;
    const auto chassis = parameter<std::string>("chassis", "ackermann",
      "Kinematic model: ackermann, differential, omnidirectional, four_wheel_steering");
    if (chassis == "ackermann") {
      config.chassis = Chassis::Ackermann;
    } else if (chassis == "differential") {
      config.chassis = Chassis::Differential;
    } else if (chassis == "omnidirectional") {
      config.chassis = Chassis::Omnidirectional;
    } else if (chassis == "four_wheel_steering") {
      config.chassis = Chassis::FourWheelSteering;
    } else {
      throw std::invalid_argument(
              "chassis must be ackermann, differential, omnidirectional or four_wheel_steering");
    }
    const int horizon = parameter<int>("mpc.horizon", 20);
    const int iterations = parameter<int>("mpc.max_iterations", 60);
    if (horizon < 2 || iterations < 1) {
      throw std::invalid_argument("Invalid MPC horizon or iteration count");
    }
    config.horizon = static_cast<std::size_t>(horizon);
    config.max_iterations = static_cast<std::size_t>(iterations);
    config.dt = parameter<double>("mpc.dt", config.dt,
      "Prediction step AND timer period, seconds > 0; horizon * dt is the lookahead time");
    config.wheelbase = parameter<double>("limits.wheelbase", config.wheelbase,
      "Ackermann axle separation, meters > 0; reference point is the rear axle center");
    config.front_axle_distance = parameter<double>(
      "limits.front_axle_distance", config.front_axle_distance,
      "4WS reference-to-front-axle distance lf, meters > 0; 4WS wheelbase = lf + lr");
    config.rear_axle_distance = parameter<double>(
      "limits.rear_axle_distance", config.rear_axle_distance,
      "4WS reference-to-rear-axle distance lr, meters > 0; default reference is axle midpoint");
    config.target_speed = parameter<double>("limits.target_speed", config.target_speed);
    config.max_vx = parameter<double>("limits.max_vx", config.max_vx);
    config.max_vy = parameter<double>("limits.max_vy", config.max_vy);
    config.max_yaw_rate = parameter<double>("limits.max_yaw_rate", config.max_yaw_rate);
    config.max_steering = parameter<double>("limits.max_steering", config.max_steering);
    config.max_rear_steering = parameter<double>("limits.max_rear_steering", config.max_rear_steering,
      "4WS rear axle equivalent steering limit, radians in (0, pi/2); positive is left");
    config.max_acceleration = parameter<double>("limits.max_acceleration", config.max_acceleration);
    config.max_lateral_acceleration = parameter<double>(
      "limits.max_lateral_acceleration", config.max_lateral_acceleration);
    config.max_yaw_acceleration = parameter<double>(
      "limits.max_yaw_acceleration", config.max_yaw_acceleration);
    config.max_steering_rate = parameter<double>(
      "limits.max_steering_rate", config.max_steering_rate);
    config.max_rear_steering_rate = parameter<double>(
      "limits.max_rear_steering_rate", config.max_rear_steering_rate,
      "4WS rear steering command slew limit, rad/s > 0; emergency stop bypasses slew limits");
    config.position_weight = parameter<double>("weights.position", config.position_weight);
    config.heading_weight = parameter<double>("weights.heading", config.heading_weight);
    config.velocity_weight = parameter<double>("weights.velocity", config.velocity_weight);
    config.effort_weight = parameter<double>("weights.effort", config.effort_weight);
    config.smoothness_weight = parameter<double>("weights.smoothness", config.smoothness_weight);
    config.terminal_weight = parameter<double>("weights.terminal", config.terminal_weight);
    config.gradient_tolerance = parameter<double>("mpc.cost_tolerance", config.gradient_tolerance);
    config.goal_position_tolerance = parameter<double>(
      "goal.position_tolerance", config.goal_position_tolerance);
    config.goal_yaw_tolerance = parameter<double>("goal.yaw_tolerance", config.goal_yaw_tolerance);
    config.progress_search_distance = parameter<double>(
      "mpc.progress_search_distance", config.progress_search_distance);
    controller_ = std::make_unique<Controller>(config);

    tracking_frame_ = parameter<std::string>("frames.tracking", "odom");
    base_frame_ = parameter<std::string>("frames.base", "base_link");
    command_type_ = parameter<std::string>("output.type", "twist",
      "twist / twist_stamped: body velocity; ackermann / four_wheel_steering: native axle angles");
    const auto command_topic = parameter<std::string>("topics.command", "cmd_vel");
    const auto path_topic = parameter<std::string>("topics.path", "plan");
    const auto odom_topic = parameter<std::string>("topics.odometry", "odom");
    const auto prediction_topic = parameter<std::string>("topics.prediction", "mpc/prediction");
    const auto status_topic = parameter<std::string>("topics.status", "mpc/status");
    const auto goal_topic = parameter<std::string>("topics.goal_reached", "mpc/goal_reached");
    odom_timeout_ = parameter<double>("safety.odom_timeout", 0.5);
    path_timeout_ = parameter<double>("safety.path_timeout", 0.0);
    transform_timeout_ = parameter<double>("safety.transform_timeout", 0.03);
    future_tolerance_ = parameter<double>("safety.future_stamp_tolerance", 0.1);
    enabled_ = parameter<bool>("enabled", true);
    const int max_points = parameter<int>("safety.max_path_points", 10000);
    if (tracking_frame_.empty() || base_frame_.empty() ||
      !std::isfinite(odom_timeout_) || odom_timeout_ <= 0.0 ||
      !std::isfinite(path_timeout_) || path_timeout_ < 0.0 ||
      !std::isfinite(transform_timeout_) || transform_timeout_ < 0.0 ||
      !std::isfinite(future_tolerance_) || future_tolerance_ < 0.0 ||
      transform_timeout_ >= config.dt || max_points < 1)
    {
      throw std::invalid_argument("Invalid frame, safety timeout or path point limit");
    }
    max_path_points_ = static_cast<std::size_t>(max_points);
    if (command_type_ == "twist") {
      twist_pub_ = create_publisher<geometry_msgs::msg::Twist>(command_topic, 1);
    } else if (command_type_ == "twist_stamped") {
      stamped_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(command_topic, 1);
    } else if (command_type_ == "ackermann") {
      if (config.chassis != Chassis::Ackermann) {
        throw std::invalid_argument("ackermann output requires ackermann chassis");
      }
      ackermann_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
        command_topic, 1);
    } else if (command_type_ == "four_wheel_steering") {
      if (config.chassis != Chassis::FourWheelSteering) {
        throw std::invalid_argument("four_wheel_steering output requires four_wheel_steering chassis");
      }
      four_wheel_pub_ = create_publisher<msg::FourWheelSteeringCommand>(command_topic, 1);
    } else {
      throw std::invalid_argument(
              "output.type must be twist, twist_stamped, ackermann or four_wheel_steering");
    }
    prediction_pub_ = create_publisher<nav_msgs::msg::Path>(prediction_topic, 1);
    status_pub_ = create_publisher<std_msgs::msg::String>(
      status_topic, rclcpp::QoS(1).transient_local());
    goal_pub_ = create_publisher<std_msgs::msg::Bool>(
      goal_topic, rclcpp::QoS(1).transient_local());
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    auto path_qos = rclcpp::QoS(1).reliable();
    if (parameter<bool>("qos.path_transient_local", false)) {
      path_qos.transient_local();
    }
    auto odom_qos = rclcpp::SensorDataQoS().keep_last(1);
    if (parameter<bool>("qos.odom_reliable", false)) {
      odom_qos.reliable();
    }
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      path_topic, path_qos, std::bind(&MpcNode::on_path, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, odom_qos, std::bind(&MpcNode::on_odom, this, std::placeholders::_1));
    enable_service_ = create_service<std_srvs::srv::SetBool>(
      "~/enable", [this](const std_srvs::srv::SetBool::Request::SharedPtr request,
      std_srvs::srv::SetBool::Response::SharedPtr response) {
        enabled_ = request->data;
        // Enabling never resumes a previously suspended trajectory.
        controller_->clear_path();
        goal_latched_ = false;
        stop(enabled_ ? "Enabled; waiting for a new path" : "Disabled", false);
        response->success = true;
        response->message = enabled_ ? "Enabled; send a new path" : "Disabled and path cleared";
      });
    clear_service_ = create_service<std_srvs::srv::Empty>(
      "~/clear_path", [this](const std_srvs::srv::Empty::Request::SharedPtr,
      std_srvs::srv::Empty::Response::SharedPtr) {
        controller_->clear_path();
        goal_latched_ = false;
        stop("Path cleared", false);
      });
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(config.dt)),
      std::bind(&MpcNode::on_timer, this));
    stop("Waiting for odometry and path", false);
    RCLCPP_INFO(get_logger(), "MPC ready: chassis=%s, frame=%s, dt=%.3f, output=%s",
      chassis.c_str(), tracking_frame_.c_str(), config.dt, command_type_.c_str());
  }

  void shutdown_stop() {stop("Shutting down", false);}

private:
  template<typename T>
  T parameter(
    const std::string & name, T value,
    const std::string & description = "See config/mpc.yaml and README for units and tuning")
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    descriptor.description = description + ". Startup-only: edit YAML and restart to change.";
    return declare_parameter<T>(name, value, descriptor);
  }

  geometry_msgs::msg::PoseStamped transform_pose(
    const geometry_msgs::msg::PoseStamped & pose)
  {
    if (pose.header.frame_id.empty()) {
      throw std::invalid_argument("Input frame_id is empty");
    }
    planar_pose(pose.pose);
    if (pose.header.frame_id == tracking_frame_) {
      return pose;
    }
    return tf_buffer_->transform(
      pose, tracking_frame_, tf2::durationFromSec(transform_timeout_));
  }

  bool stamp_valid(
    const builtin_interfaces::msg::Time & stamp, double timeout, bool allow_zero) const
  {
    // 消息自身时间戳防止“不断重发旧数据”，steady 接收间隔另防止 /clock 暂停掩盖失联。
    if (stamp.sec < 0 || stamp.nanosec >= 1000000000U) {
      return false;
    }
    const rclcpp::Time time(stamp, get_clock()->get_clock_type());
    if (time.nanoseconds() == 0) {
      return allow_zero;
    }
    const double age = (now() - time).seconds();
    return age >= -future_tolerance_ && (timeout == 0.0 || age <= timeout);
  }

  void on_path(const nav_msgs::msg::Path::SharedPtr message)
  {
    if (!enabled_) {
      stop("Disabled; path ignored", false);
      return;
    }
    // 新任务即使无效也取消旧任务；不能输入报错后仍沿着旧轨迹继续行驶。
    controller_->clear_path();
    goal_latched_ = false;
    if (message->poses.empty()) {
      stop("Empty path: cancelled", false);
      return;
    }
    try {
      if (message->poses.size() > max_path_points_) {
        throw std::invalid_argument("Path exceeds safety.max_path_points");
      }
      if (message->header.frame_id.empty() ||
        !stamp_valid(message->header.stamp, path_timeout_, path_timeout_ == 0.0))
      {
        throw std::invalid_argument("Path has empty frame, stale, zero or future timestamp");
      }
      geometry_msgs::msg::TransformStamped transform;
      const bool needs_transform = message->header.frame_id != tracking_frame_;
      if (needs_transform) {
        transform = tf_buffer_->lookupTransform(
          tracking_frame_, message->header.frame_id,
          rclcpp::Time(message->header.stamp, get_clock()->get_clock_type()),
          rclcpp::Duration::from_seconds(transform_timeout_));
      }
      std::vector<Pose> poses;
      poses.reserve(message->poses.size());
      for (const auto & p : message->poses) {
        if (!p.header.frame_id.empty() && p.header.frame_id != message->header.frame_id) {
          throw std::invalid_argument("Path pose frame differs from Path.header.frame_id");
        }
        planar_pose(p.pose);
        geometry_msgs::msg::PoseStamped transformed;
        if (needs_transform) {
          tf2::doTransform(p, transformed, transform);
          poses.push_back(planar_pose(transformed.pose));
        } else {
          poses.push_back(planar_pose(p.pose));
        }
      }
      controller_->set_path(poses);
      path_received_ = SteadyClock::now();
      path_stamp_ = message->header.stamp;
      publish_goal(false);
      set_status("Path accepted");
    } catch (const tf2::TransformException & error) {
      stop(std::string("Path TF failure: ") + error.what(), true);
    } catch (const std::invalid_argument & error) {
      stop(std::string("Invalid path: ") + error.what(), true);
    }
  }

  void on_odom(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    try {
      if (message->child_frame_id != base_frame_) {
        throw std::invalid_argument("Odometry.child_frame_id must match frames.base");
      }
      if (!stamp_valid(message->header.stamp, odom_timeout_, false)) {
        throw std::invalid_argument("Odometry timestamp is zero, stale or in the future");
      }
      planar_pose(message->pose.pose);
      odometry_ = message;
      odom_received_ = SteadyClock::now();
    } catch (const std::invalid_argument & error) {
      odometry_.reset();
      stop(std::string("Invalid odometry: ") + error.what(), true);
    }
  }

  void on_timer()
  {
    // 顺序不能颠倒：使能/时效检查 -> 同帧位姿 -> MPC -> 截止时间复查 -> 发布。
    // 单线程 executor 串行执行回调，避免路径更新与求解同时修改核心状态。
    const auto tick = SteadyClock::now();
    const double ros_time = now().seconds();
    if (last_ros_time_ >= 0.0 && ros_time < last_ros_time_) {
      controller_->clear_path();
      odometry_.reset();
      goal_latched_ = false;
      last_ros_time_ = ros_time;
      stop("ROS time moved backwards; send fresh odometry and path", true);
      return;
    }
    last_ros_time_ = ros_time;
    if (!enabled_) {
      stop("Disabled", false);
      return;
    }
    if (!odometry_ || std::chrono::duration<double>(tick - odom_received_).count() > odom_timeout_ ||
      !stamp_valid(odometry_->header.stamp, odom_timeout_, false))
    {
      stop("Odometry missing, stale or invalid timestamp", true);
      return;
    }
    if (!controller_->has_path()) {
      stop(goal_latched_ ? "Goal reached" : "Waiting for path", false);
      return;
    }
    if (path_timeout_ > 0.0 &&
      (std::chrono::duration<double>(tick - path_received_).count() > path_timeout_ ||
      !stamp_valid(path_stamp_, path_timeout_, false)))
    {
      controller_->clear_path();
      stop("Path expired; send a new path", true);
      return;
    }
    try {
      geometry_msgs::msg::PoseStamped input;
      input.header = odometry_->header;
      input.pose = odometry_->pose.pose;
      const auto pose = planar_pose(transform_pose(input).pose);
      const auto result = controller_->compute(pose, previous_);
      const double elapsed = std::chrono::duration<double>(SteadyClock::now() - tick).count();
      if (!result.valid || elapsed >= controller_->config().dt) {
        stop(result.valid ? "Control deadline exceeded; command discarded" :
          "MPC failure: " + result.message, true);
        return;
      }
      if (std::chrono::duration<double>(SteadyClock::now() - odom_received_).count() >
        odom_timeout_ || !stamp_valid(odometry_->header.stamp, odom_timeout_, false))
      {
        stop("Odometry expired during solve", true);
        return;
      }
      nav_msgs::msg::Path prediction;
      prediction.header.frame_id = tracking_frame_;
      prediction.header.stamp = now();
      for (const auto & p : result.prediction) {
        geometry_msgs::msg::PoseStamped item;
        item.header = prediction.header;
        item.pose = to_message(p);
        prediction.poses.push_back(item);
      }
      prediction_pub_->publish(prediction);
      if (result.goal_reached) {
        goal_latched_ = true;
        controller_->clear_path();
        stop("Goal reached", false);
      } else {
        publish_command(result.command);
        publish_goal(false);
        set_status(result.converged ? "Tracking" : "Tracking: feasible solution, not converged");
      }
    } catch (const tf2::TransformException & error) {
      stop(std::string("Odometry TF failure: ") + error.what(), true);
    } catch (const std::invalid_argument & error) {
      stop(std::string("Invalid control input: ") + error.what(), true);
    }
  }

  void publish_command(const Control & control)
  {
    // Twist 表示“参考点的本体速度”，不是舵机角度。
    // 四轮转向的 vy 必须由两轴转角推导，直接使用 Control.vy 会丢失蟹行分量。
    geometry_msgs::msg::Twist twist;
    twist.linear.x = control.vx;
    twist.linear.y = lateral_velocity(control, controller_->config());
    twist.angular.z = yaw_rate(control, controller_->config());
    if (twist_pub_) {
      twist_pub_->publish(twist);
    } else if (stamped_pub_) {
      geometry_msgs::msg::TwistStamped message;
      message.header.stamp = now();
      message.header.frame_id = base_frame_;
      message.twist = twist;
      stamped_pub_->publish(message);
    }
    else if (ackermann_pub_) {
      ackermann_msgs::msg::AckermannDriveStamped message;
      message.header.stamp = now();
      message.header.frame_id = base_frame_;
      message.drive.speed = static_cast<float>(control.vx);
      message.drive.steering_angle = static_cast<float>(control.turn);
      ackermann_pub_->publish(message);
    } else if (four_wheel_pub_) {
      msg::FourWheelSteeringCommand message;
      message.header.stamp = now();
      message.header.frame_id = base_frame_;
      message.longitudinal_velocity = control.vx;
      message.front_steering_angle = control.turn;
      message.rear_steering_angle = control.rear_turn;
      four_wheel_pub_->publish(message);
    }
    // 限斜率基于上次真正发布的控制量，而不是预测序列末端或里程计 twist。
    previous_ = control;
  }

  void publish_goal(bool reached)
  {
    std_msgs::msg::Bool message;
    message.data = reached;
    goal_pub_->publish(message);
  }

  void set_status(const std::string & status, bool warning = false)
  {
    if (status != last_status_) {
      if (warning) {
        RCLCPP_WARN(get_logger(), "%s", status.c_str());
      } else {
        RCLCPP_INFO(get_logger(), "%s", status.c_str());
      }
      last_status_ = status;
    }
    std_msgs::msg::String message;
    message.data = status;
    status_pub_->publish(message);
  }

  void stop(const std::string & reason, bool warning)
  {
    // 急停请求有意绕过正常限斜率：速度及两轴转角均置零，由驱动负责安全执行。
    publish_command({});
    publish_goal(goal_latched_);
    set_status(reason, warning);
  }

  std::unique_ptr<Controller> controller_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr stamped_pub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_pub_;
  rclcpp::Publisher<msg::FourWheelSteeringCommand>::SharedPtr four_wheel_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr prediction_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goal_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr clear_service_;
  rclcpp::TimerBase::SharedPtr timer_;
  nav_msgs::msg::Odometry::SharedPtr odometry_;
  builtin_interfaces::msg::Time path_stamp_;
  SteadyClock::time_point odom_received_;
  SteadyClock::time_point path_received_;
  Control previous_;
  std::string tracking_frame_, base_frame_, command_type_, last_status_;
  double odom_timeout_, path_timeout_, transform_timeout_, future_tolerance_;
  double last_ros_time_{-1.0};
  std::size_t max_path_points_;
  bool enabled_{true};
  bool goal_latched_{false};
};
}  // namespace path_follow_mpc

#ifndef MPC_NO_MAIN
namespace
{
volatile std::sig_atomic_t stop_requested = 0;
void request_stop(int) {stop_requested = 1;}
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, request_stop);
  std::signal(SIGTERM, request_stop);
  int exit_code = 0;
  try {
    auto node = std::make_shared<path_follow_mpc::MpcNode>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    while (rclcpp::ok() && !stop_requested) {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    if (rclcpp::ok()) {
      node->shutdown_stop();
    }
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("mpc_tracker"), "%s", error.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
#endif
