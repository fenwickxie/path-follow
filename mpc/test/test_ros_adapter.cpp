#define MPC_NO_MAIN
#include "../src/mpc_node.cpp"

#include <iostream>
#include <limits>

#include "tf2_ros/static_transform_broadcaster.h"

namespace
{
void require(bool condition, const std::string & message)
{
  if (!condition) {
    throw std::runtime_error(message);
  }
}

struct Harness
{
  explicit Harness(
    const std::string & name, const std::string & output = "twist",
    double path_timeout = 0.0, bool transient_path = false,
    const std::string & chassis = "omnidirectional")
  {
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "-r", "__ns:=/" + name});
    options.parameter_overrides({
      {"chassis", chassis},
      {"output.type", output},
      {"safety.path_timeout", path_timeout},
      {"safety.odom_timeout", 0.3},
      {"qos.path_transient_local", transient_path}});
    tracker = std::make_shared<path_follow_mpc::MpcNode>(options);
    driver = std::make_shared<rclcpp::Node>("test_driver", "/" + name);
    executor.add_node(tracker);
    executor.add_node(driver);
    auto path_qos = rclcpp::QoS(1).reliable();
    if (transient_path) {
      path_qos.transient_local();
    }
    path_pub = driver->create_publisher<nav_msgs::msg::Path>("plan", path_qos);
    odom_pub = driver->create_publisher<nav_msgs::msg::Odometry>(
      "odom", rclcpp::SensorDataQoS().keep_last(1));
    status_sub = driver->create_subscription<std_msgs::msg::String>(
      "mpc/status", rclcpp::QoS(1).transient_local(),
      [this](std_msgs::msg::String::SharedPtr message) {status = message->data;});
    goal_sub = driver->create_subscription<std_msgs::msg::Bool>(
      "mpc/goal_reached", rclcpp::QoS(1).transient_local(),
      [this](std_msgs::msg::Bool::SharedPtr message) {goal = message->data;});
    prediction_sub = driver->create_subscription<nav_msgs::msg::Path>(
      "mpc/prediction", 1, [this](nav_msgs::msg::Path::SharedPtr message) {
        prediction_frame = message->header.frame_id;
        prediction_size = message->poses.size();
      });
    model.chassis = chassis == "ackermann" ? path_follow_mpc::Chassis::Ackermann :
      (chassis == "four_wheel_steering" ? path_follow_mpc::Chassis::FourWheelSteering :
      path_follow_mpc::Chassis::Omnidirectional);
    if (output == "twist") {
      twist_sub = driver->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 1, [this](geometry_msgs::msg::Twist::SharedPtr message) {
          command = *message;
          ++commands;
        });
    } else if (output == "twist_stamped") {
      stamped_sub = driver->create_subscription<geometry_msgs::msg::TwistStamped>(
        "cmd_vel", 1, [this](geometry_msgs::msg::TwistStamped::SharedPtr message) {
          command = message->twist;
          command_frame = message->header.frame_id;
          ++commands;
        });
    } else if (output == "ackermann") {
      ackermann_sub = driver->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
        "cmd_vel", 1, [this](ackermann_msgs::msg::AckermannDriveStamped::SharedPtr message) {
          native_control = {message->drive.speed, 0.0, message->drive.steering_angle, 0.0};
          command_frame = message->header.frame_id;
          command_stamp = message->header.stamp;
          update_native_command();
        });
    } else if (output == "four_wheel_steering") {
      four_wheel_sub = driver->create_subscription<path_follow_mpc::msg::FourWheelSteeringCommand>(
        "cmd_vel", 1, [this](path_follow_mpc::msg::FourWheelSteeringCommand::SharedPtr message) {
          native_control = {message->longitudinal_velocity, 0.0,
            message->front_steering_angle, message->rear_steering_angle};
          command_frame = message->header.frame_id;
          command_stamp = message->header.stamp;
          update_native_command();
        });
    } else {
      throw std::invalid_argument("Test harness output is not supported");
    }
    enable = driver->create_client<std_srvs::srv::SetBool>("mpc_tracker/enable");
    clear = driver->create_client<std_srvs::srv::Empty>("mpc_tracker/clear_path");
    spin_for(0.25, false);
    require(path_pub->get_subscription_count() > 0 &&
      odom_pub->get_subscription_count() > 0, "ROS discovery");
  }

  void odom(const std::string & frame = "odom", bool stale = false)
  {
    nav_msgs::msg::Odometry message;
    message.header.frame_id = frame;
    message.header.stamp = driver->now() - rclcpp::Duration::from_seconds(stale ? 5.0 : 0.0);
    message.child_frame_id = "base_link";
    message.pose.pose.orientation.w = 1.0;
    odom_pub->publish(message);
  }

  void path(const std::string & frame = "odom", bool invalid = false, bool single = false)
  {
    nav_msgs::msg::Path message;
    message.header.frame_id = frame;
    message.header.stamp = driver->now();
    geometry_msgs::msg::PoseStamped a, b;
    a.pose.orientation.w = 1.0;
    b.pose.orientation.w = 1.0;
    b.pose.position.x = invalid ? std::numeric_limits<double>::quiet_NaN() : 3.0;
    message.poses = single ? std::vector<geometry_msgs::msg::PoseStamped>{a} :
      std::vector<geometry_msgs::msg::PoseStamped>{a, b};
    path_pub->publish(message);
  }

  void diagonal_path()
  {
    nav_msgs::msg::Path message;
    message.header.frame_id = "odom";
    message.header.stamp = driver->now();
    geometry_msgs::msg::PoseStamped a, b;
    a.pose.orientation.w = 1.0;
    b.pose.orientation.w = 1.0;
    b.pose.position.x = 3.0;
    b.pose.position.y = 0.24;
    message.poses = {a, b};
    path_pub->publish(message);
  }

  void update_native_command()
  {
    command.linear.x = native_control.vx;
    command.linear.y = path_follow_mpc::lateral_velocity(native_control, model);
    command.angular.z = path_follow_mpc::yaw_rate(native_control, model);
    ++commands;
  }

  void spin_for(double seconds, bool feed_odom, const std::string & frame = "odom")
  {
    const auto end = SteadyClock::now() + std::chrono::duration<double>(seconds);
    auto next = SteadyClock::now();
    while (SteadyClock::now() < end) {
      if (feed_odom && SteadyClock::now() >= next) {
        odom(frame);
        next = SteadyClock::now() + std::chrono::milliseconds(30);
      }
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    executor.spin_some();
  }

  bool stopped() const
  {
    return command.linear.x == 0.0 && command.linear.y == 0.0 && command.angular.z == 0.0;
  }

  void set_enabled(bool value)
  {
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = value;
    auto future = enable->async_send_request(request);
    require(executor.spin_until_future_complete(future, std::chrono::seconds(2)) ==
      rclcpp::FutureReturnCode::SUCCESS && future.get()->success, "enable service");
  }

  using SteadyClock = std::chrono::steady_clock;
  std::shared_ptr<path_follow_mpc::MpcNode> tracker;
  rclcpp::Node::SharedPtr driver;
  rclcpp::executors::SingleThreadedExecutor executor;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr stamped_sub;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_sub;
  rclcpp::Subscription<path_follow_mpc::msg::FourWheelSteeringCommand>::SharedPtr four_wheel_sub;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr goal_sub;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr prediction_sub;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr enable;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr clear;
  geometry_msgs::msg::Twist command;
  path_follow_mpc::Config model;
  path_follow_mpc::Control native_control;
  builtin_interfaces::msg::Time command_stamp;
  std::string status, command_frame, prediction_frame;
  std::size_t prediction_size{0};
  int commands{0};
  bool goal{false};
};

void test_safety()
{
  Harness h("safety");
  require(h.commands > 0 && h.stopped(), "startup must publish stop");
  h.path();
  h.spin_for(0.35, true);
  require(h.command.linear.x > 0.0, "fresh path and odom must drive");
  require(h.prediction_size == 21 && h.prediction_frame == "odom", "prediction contract");
  h.spin_for(0.5, false);
  require(h.stopped() && h.status.find("Odometry") != std::string::npos, "odom watchdog");
  h.spin_for(0.25, true);
  require(h.command.linear.x > 0.0, "resume after fresh odometry");
  h.set_enabled(false);
  h.spin_for(0.15, true);
  require(h.stopped(), "disable must stop");
  h.path();
  h.spin_for(0.15, true);
  h.set_enabled(true);
  h.spin_for(0.15, true);
  require(h.stopped(), "enable requires a new path");
  h.path();
  h.spin_for(0.25, true);
  require(h.command.linear.x > 0.0, "new path after enable");
  auto future = h.clear->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>());
  require(h.executor.spin_until_future_complete(future, std::chrono::seconds(2)) ==
    rclcpp::FutureReturnCode::SUCCESS, "clear service response");
  h.spin_for(0.15, true);
  require(h.stopped(), "clear path must stop");
  h.path();
  h.spin_for(0.2, true);
  nav_msgs::msg::Path empty;
  h.path_pub->publish(empty);
  h.spin_for(0.15, true);
  require(h.stopped(), "empty path must cancel");
  h.path("odom", true);
  h.spin_for(0.15, true);
  require(h.stopped(), "invalid path must stop");
  h.path("odom", false, true);
  h.spin_for(0.25, true);
  require(h.goal && h.stopped(), "goal must latch and stop");
  h.path();
  h.spin_for(0.25, true);
  require(!h.goal && !h.stopped(), "new path resets goal latch");
  h.odom("odom", true);
  h.spin_for(0.12, false);
  require(h.stopped(), "stale stamped odometry rejected");
  nav_msgs::msg::Odometry invalid_stamp;
  invalid_stamp.header.frame_id = "odom";
  invalid_stamp.header.stamp.sec = -1;
  invalid_stamp.child_frame_id = "base_link";
  invalid_stamp.pose.pose.orientation.w = 1.0;
  h.odom_pub->publish(invalid_stamp);
  h.spin_for(0.12, false);
  require(h.stopped(), "negative timestamp rejected without crashing");
  h.path("missing_frame");
  h.spin_for(0.15, true);
  require(h.stopped(), "missing path TF must stop");
}

void test_tf_and_stamped()
{
  Harness h("tf_stamped", "twist_stamped", 0.0, true);
  tf2_ros::StaticTransformBroadcaster broadcaster(h.driver);
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = h.driver->now();
  transform.header.frame_id = "odom";
  transform.child_frame_id = "test_map";
  transform.transform.rotation.w = 1.0;
  transform.transform.translation.y = 0.4;
  broadcaster.sendTransform(transform);
  h.spin_for(0.2, true, "test_map");
  h.path("test_map");
  h.spin_for(0.35, true, "test_map");
  require(h.command.linear.x > 0.0, "transformed path and odometry track");
  require(h.command_frame == "base_link", "stamped command is in body frame");
  require(h.prediction_frame == "odom", "prediction uses tracking frame");
  h.spin_for(0.4, true, "missing_odom_frame");
  require(h.stopped(), "missing odometry TF must stop");
}

void test_path_timeout()
{
  Harness h("path_timeout", "twist", 0.35);
  h.path();
  h.spin_for(0.2, true);
  require(!h.stopped(), "path within TTL drives");
  h.spin_for(0.4, true);
  require(h.stopped(), "expired path must stop");
  h.spin_for(0.2, true);
  require(h.stopped(), "expired path cannot resume without replacement");
}

void test_configuration()
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"chassis", "hovercraft"}});
  bool caught = false;
  try {
    auto node = std::make_shared<path_follow_mpc::MpcNode>(options);
  } catch (const std::invalid_argument &) {
    caught = true;
  }
  require(caught, "invalid chassis rejected at startup");
  // 消息类型与模型不匹配时应启动失败，不能静默丢掉后轴转角。
  for (const auto & pair : std::vector<std::pair<std::string, std::string>>{
      {"four_wheel_steering", "ackermann"}, {"ackermann", "four_wheel_steering"}})
  {
    options.parameter_overrides({{"chassis", pair.first}, {"output.type", pair.second}});
    caught = false;
    try {
      auto node = std::make_shared<path_follow_mpc::MpcNode>(options);
    } catch (const std::invalid_argument &) {
      caught = true;
    }
    require(caught, "mismatched native output rejected");
  }
}

void test_native_output(const std::string & chassis)
{
  Harness h("native_" + chassis, chassis, 0.0, false, chassis);
  require(h.commands > 0 && h.stopped(), "native output startup stop");
  h.diagonal_path();
  h.spin_for(0.45, true);
  require(h.native_control.vx > 0.0 && h.native_control.turn > 0.001,
    "native speed and front steering are populated");
  require(h.command_frame == "base_link" && h.command_stamp.sec > 0,
    "native command stamped body-frame header");
  require(std::abs(h.native_control.turn) <= h.model.max_steering, "native front angle bound");
  if (chassis == "four_wheel_steering") {
    require(h.native_control.rear_turn > 0.001, "native message retains crab rear steering");
    require(h.command.linear.y > 0.0, "native crab has derived lateral velocity");
  } else {
    const double expected = h.native_control.vx * std::tan(h.native_control.turn) /
      h.model.wheelbase;
    require(std::abs(expected - h.native_control.turn) > 0.001,
      "Ackermann field is an angle, not yaw rate");
  }
  h.set_enabled(false);
  h.spin_for(0.12, true);
  require(h.stopped() && h.native_control.turn == 0.0 && h.native_control.rear_turn == 0.0,
    "native disable clears speed and steering");
  h.set_enabled(true);
  h.diagonal_path();
  h.spin_for(0.2, true);
  require(!h.stopped(), "native resumes only on new path");
  h.spin_for(0.5, false);
  require(h.stopped() && h.native_control.turn == 0.0 && h.native_control.rear_turn == 0.0,
    "native odom watchdog clears all controls");
  h.path("odom", false, true);
  h.spin_for(0.2, true);
  require(h.goal && h.stopped(), "native goal stops");
}

void test_four_wheel_twist(const std::string & output)
{
  Harness h("four_" + output, output, 0.0, false, "four_wheel_steering");
  h.diagonal_path();
  h.spin_for(0.35, true);
  require(h.command.linear.x > 0.0 && h.command.linear.y > 0.001,
    "four wheel Twist must contain derived crab vy");
  require(std::abs(h.command.angular.z) < 0.15, "crab angular.z is yaw rate, not axle angle");
  require(std::abs(h.command.linear.y) <= h.model.max_vy &&
    std::abs(h.command.angular.z) <= h.model.max_yaw_rate, "four wheel Twist limits");
  h.set_enabled(false);
  h.spin_for(0.12, true);
  require(h.stopped(), "four wheel Twist disable stop");
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    test_configuration();
    test_safety();
    test_tf_and_stamped();
    test_path_timeout();
    test_native_output("ackermann");
    test_native_output("four_wheel_steering");
    test_four_wheel_twist("twist");
    test_four_wheel_twist("twist_stamped");
    std::cout << "All ROS adapter tests passed\n";
  } catch (const std::exception & error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
