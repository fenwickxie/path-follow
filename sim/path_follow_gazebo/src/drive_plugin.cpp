#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <path_follow_mpc/msg/four_wheel_steering_command.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace path_follow_gazebo
{
namespace
{
constexpr double pi = 3.14159265358979323846;
double wrap(double a) {return std::atan2(std::sin(a), std::cos(a));}

template<typename T>
T read(const sdf::ElementPtr & sdf, const std::string & key, const T & fallback)
{
  return sdf->HasElement(key) ? sdf->Get<T>(key) : fallback;
}

struct Command
{
  double vx{0}, vy{0}, w{0};
  double front{0}, rear{0};
  bool native{false};
};
}  // namespace

// Only joint motors are actuated. Contact forces and the physics engine determine
// the chassis pose; no SetWorldPose / SetLinearVel shortcut is used.
class DrivePlugin : public gazebo::ModelPlugin
{
public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    node_ = gazebo_ros::Node::Get(sdf);
    try {
      if (model_->GetWorld()->Physics()->GetType() != "ode") {
        throw std::invalid_argument("the physical wheel drive requires the ODE engine");
      }
      body_link_ = model_->GetLink("base_link");
      if (!body_link_) {
        throw std::invalid_argument("model requires a base_link rigid body");
      }
      chassis_ = read<std::string>(sdf, "chassis", "ackermann");
      type_ = read<std::string>(sdf, "command_type", "twist");
      frame_ = read<std::string>(sdf, "odom_frame", "odom");
      base_ = read<std::string>(sdf, "base_frame", "base_link");
      radius_ = read<double>(sdf, "wheel_radius", 0.1);
      track_ = read<double>(sdf, "track", 0.5);
      lf_ = read<double>(sdf, "front_axle_distance", 0.325);
      lr_ = read<double>(sdf, "rear_axle_distance", 0.325);
      timeout_ = read<double>(sdf, "command_timeout", 0.5);
      torque_ = read<double>(sdf, "wheel_torque", 8.0);
      steer_torque_ = read<double>(sdf, "steering_torque", 10.0);
      steer_rate_ = read<double>(sdf, "steering_rate", 2.0);
      steer_limit_ = read<double>(sdf, "steering_limit", 0.8);
      max_speed_ = read<double>(sdf, "max_speed", 2.0);
      max_yaw_ = read<double>(sdf, "max_yaw_rate", 2.0);
      period_ = 1.0 / read<double>(sdf, "publish_rate", 50.0);
      const std::array<std::string, 4> valid{
        "ackermann", "differential", "omnidirectional", "four_wheel_steering"};
      if (std::find(valid.begin(), valid.end(), chassis_) == valid.end()) {
        throw std::invalid_argument("unknown chassis: " + chassis_);
      }
      for (double v : {radius_, track_, lf_, timeout_, torque_, steer_torque_,
          steer_rate_, steer_limit_, max_speed_, max_yaw_, period_})
      {
        if (!std::isfinite(v) || v <= 0) {
          throw std::invalid_argument("all geometry, limits and rates must be finite and positive");
        }
      }
      if (!std::isfinite(lr_) || lr_ < 0 || (chassis_ != "ackermann" && lr_ == 0)) {
        throw std::invalid_argument("rear_axle_distance must be positive (zero for rear-axle Ackermann)");
      }
      if (steer_limit_ >= pi / 2) {
        if (chassis_ != "omnidirectional" || steer_limit_ > pi) {
          throw std::invalid_argument("steering_limit must be < pi/2 except for omni, where it must be <= pi");
        }
      }
      const std::array<std::string, 4> names{"front_left", "front_right", "rear_left", "rear_right"};
      for (std::size_t i = 0; i < names.size(); ++i) {
        if (chassis_ == "differential" && i >= 2) {
          continue;
        }
        wheels_[i] = model_->GetJoint(names[i] + "_wheel_joint");
        steer_[i] = model_->GetJoint(names[i] + "_steer_joint");
        const bool steerable = chassis_ == "omnidirectional" ||
          chassis_ == "four_wheel_steering" || (chassis_ == "ackermann" && i < 2);
        if (!wheels_[i] || (steerable && !steer_[i])) {
          throw std::invalid_argument("missing drive/steering joint for " + names[i]);
        }
      }
      const auto topic = read<std::string>(sdf, "command_topic", "cmd_vel");
      const auto qos = rclcpp::QoS(1);
      if (type_ == "twist") {
        sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
          topic, qos, [this](geometry_msgs::msg::Twist::ConstSharedPtr m) {
            Receive({m->linear.x, m->linear.y, m->angular.z});
          });
      } else if (type_ == "twist_stamped") {
        sub_ = node_->create_subscription<geometry_msgs::msg::TwistStamped>(
          topic, qos, [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr m) {
            Receive({m->twist.linear.x, m->twist.linear.y, m->twist.angular.z});
          });
      } else if (type_ == "ackermann" && chassis_ == "ackermann") {
        sub_ = node_->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
          topic, qos, [this](ackermann_msgs::msg::AckermannDriveStamped::ConstSharedPtr m) {
            const double a = m->drive.steering_angle;
            if (!std::isfinite(a) || std::abs(a) > steer_limit_) {
              Reject("invalid Ackermann steering angle");
              return;
            }
            Receive({m->drive.speed, 0, m->drive.speed * std::tan(a) / (lf_ + lr_), a, 0, true});
          });
      } else if (type_ == "four_wheel_steering" && chassis_ == "four_wheel_steering") {
        sub_ = node_->create_subscription<path_follow_mpc::msg::FourWheelSteeringCommand>(
          topic, qos, [this](path_follow_mpc::msg::FourWheelSteeringCommand::ConstSharedPtr m) {
            const double f = m->front_steering_angle, r = m->rear_steering_angle;
            if (!std::isfinite(f) || !std::isfinite(r) ||
              std::abs(f) > steer_limit_ || std::abs(r) > steer_limit_)
            {
              Reject("invalid four-wheel steering angles");
              return;
            }
            const double v = m->longitudinal_velocity;
            Receive({v, v * (lr_ * std::tan(f) + lf_ * std::tan(r)) / (lf_ + lr_),
              v * (std::tan(f) - std::tan(r)) / (lf_ + lr_), f, r, true});
          });
      } else {
        throw std::invalid_argument("command_type is unknown or incompatible with chassis");
      }
      odom_ = node_->create_publisher<nav_msgs::msg::Odometry>(
        read<std::string>(sdf, "odometry_topic", "odom"), 10);
      joints_ = node_->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
      tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
      last_publish_ = model_->GetWorld()->SimTime().Double();
      update_ = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&DrivePlugin::Update, this, std::placeholders::_1));
      RCLCPP_INFO(node_->get_logger(), "Physical joint drive ready: %s (%s)",
        chassis_.c_str(), type_.c_str());
    } catch (const std::exception & e) {
      RCLCPP_FATAL(node_->get_logger(), "Gazebo drive initialization failed: %s", e.what());
      throw;
    }
  }

  void Reset() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    command_ = {};
    received_ = false;
    last_publish_ = 0;
  }

private:
  void Reject(const char * reason)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      command_ = {};
      received_ = false;
    }
    RCLCPP_ERROR(node_->get_logger(), "%s; stopping wheel motors", reason);
  }

  void Receive(Command c)
  {
    if (!std::isfinite(c.vx) || !std::isfinite(c.vy) || !std::isfinite(c.w) ||
      std::abs(c.vx) > max_speed_ || std::abs(c.vy) > max_speed_ || std::abs(c.w) > max_yaw_)
    {
      Reject("non-finite or out-of-range body command");
      return;
    }
    if ((chassis_ == "ackermann" || chassis_ == "differential") && std::abs(c.vy) > 1e-6) {
      Reject("this chassis does not support lateral velocity at its control point");
      return;
    }
    if ((chassis_ == "ackermann" || chassis_ == "four_wheel_steering") &&
      std::abs(c.vx) < 1e-6 && (std::abs(c.vy) > 1e-6 || std::abs(c.w) > 1e-6))
    {
      Reject("steering chassis cannot rotate or translate sideways at zero longitudinal speed");
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    command_ = c;
    last_command_ = node_->get_clock()->now();
    received_ = true;
  }

  void Update(const gazebo::common::UpdateInfo & info)
  {
    Command c;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const double age = (node_->get_clock()->now() - last_command_).seconds();
      if (received_ && age >= 0 && age <= timeout_) {
        c = command_;
      } else if (received_) {
        received_ = false;
        command_ = {};
        RCLCPP_WARN(node_->get_logger(), "Command timeout or clock reset; braking wheel motors");
      }
    }
    const bool stop = std::hypot(c.vx, c.vy) + std::abs(c.w) < 1e-8;
    for (std::size_t i = 0; i < wheels_.size(); ++i) {
      if (!wheels_[i]) {
        continue;
      }
      const double x = chassis_ == "differential" ? 0 : (i < 2 ? lf_ : -lr_);
      const double y = (i % 2 == 0 ? 1 : -1) * track_ / 2;
      const double vx = c.vx - c.w * y, vy = c.vy + c.w * x;
      double angle = 0, speed = vx;
      if (steer_[i]) {
        speed = std::hypot(vx, vy);
        angle = stop ? (c.native ? (i < 2 ? c.front : c.rear) : 0) : std::atan2(vy, vx);
        // Reverse wheel rotation rather than requesting a 180-degree steering flip.
        if (std::abs(wrap(angle)) > pi / 2) {
          angle = wrap(angle + pi);
          speed = -speed;
        }
        if (chassis_ != "omnidirectional") {
          if (std::abs(angle) > steer_limit_) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
              "Per-wheel steering saturated at its physical joint limit");
          }
          angle = std::clamp(angle, -steer_limit_, steer_limit_);
        }
        const double error = wrap(angle - steer_[i]->Position(0));
        steer_[i]->SetParam("fmax", 0, steer_torque_);
        steer_[i]->SetParam("vel", 0, std::clamp(10 * error, -steer_rate_, steer_rate_));
        // Do not scrub the ground while the steering motor is still aligning.
        speed *= std::max(0.0, std::cos(error));
      }
      wheels_[i]->SetParam("fmax", 0, torque_);
      wheels_[i]->SetParam("vel", 0, stop ? 0.0 : speed / radius_);
    }
    const double now = info.simTime.Double();
    if (now < last_publish_) {
      Reset();
    }
    if (now - last_publish_ < period_) {
      return;
    }
    last_publish_ = now;
    Publish(info.simTime);
  }

  void Publish(const gazebo::common::Time & time)
  {
    const auto pose = model_->WorldPose();
    const auto world_angular = body_link_->WorldAngularVel();
    // The model origin is the control point, not necessarily the chassis CoG.
    const auto offset = pose.Pos() - body_link_->WorldPose().Pos();
    const auto velocity = pose.Rot().RotateVectorReverse(
      body_link_->WorldLinearVel() + world_angular.Cross(offset));
    const auto angular = pose.Rot().RotateVectorReverse(world_angular);
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = rclcpp::Time(time.sec, time.nsec);
    odom.header.frame_id = frame_;
    odom.child_frame_id = base_;
    odom.pose.pose.position.x = pose.Pos().X();
    odom.pose.pose.position.y = pose.Pos().Y();
    odom.pose.pose.position.z = pose.Pos().Z();
    odom.pose.pose.orientation.x = pose.Rot().X();
    odom.pose.pose.orientation.y = pose.Rot().Y();
    odom.pose.pose.orientation.z = pose.Rot().Z();
    odom.pose.pose.orientation.w = pose.Rot().W();
    odom.twist.twist.linear.x = velocity.X();
    odom.twist.twist.linear.y = velocity.Y();
    odom.twist.twist.angular.z = angular.Z();
    odom_->publish(odom);
    geometry_msgs::msg::TransformStamped tf;
    tf.header = odom.header;
    tf.child_frame_id = base_;
    tf.transform.translation.x = pose.Pos().X();
    tf.transform.translation.y = pose.Pos().Y();
    tf.transform.translation.z = pose.Pos().Z();
    tf.transform.rotation = odom.pose.pose.orientation;
    tf_->sendTransform(tf);
    sensor_msgs::msg::JointState state;
    state.header = odom.header;
    for (std::size_t i = 0; i < wheels_.size(); ++i) {
      for (const auto & joint : {wheels_[i], steer_[i]}) {
        if (joint) {
          state.name.push_back(joint->GetName());
          state.position.push_back(joint->Position(0));
          state.velocity.push_back(joint->GetVelocity(0));
        }
      }
    }
    joints_->publish(state);
  }

  gazebo::physics::ModelPtr model_;
  gazebo::physics::LinkPtr body_link_;
  gazebo_ros::Node::SharedPtr node_;
  gazebo::event::ConnectionPtr update_;
  std::array<gazebo::physics::JointPtr, 4> wheels_, steer_;
  rclcpp::SubscriptionBase::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joints_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  std::mutex mutex_;
  Command command_;
  rclcpp::Time last_command_{0, 0, RCL_ROS_TIME};
  bool received_{false};
  std::string chassis_, type_, frame_, base_;
  double radius_, track_, lf_, lr_, timeout_, torque_, steer_torque_, steer_rate_;
  double steer_limit_, max_speed_, max_yaw_, period_, last_publish_{0};
};

GZ_REGISTER_MODEL_PLUGIN(DrivePlugin)
}  // namespace path_follow_gazebo
