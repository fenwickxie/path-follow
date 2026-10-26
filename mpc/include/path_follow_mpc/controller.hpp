#pragma once

#include <array>
#include <cstddef>
#include <string>
#include <vector>

namespace path_follow_mpc
{

enum class Chassis { Ackermann, Differential, Omnidirectional, FourWheelSteering };

/// 平面位姿：x/y 单位 m，yaw 单位 rad；所有输入路径和测量必须在同一固定坐标系。
struct Pose
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

/// 控制量均以 frames.base 为参考；x 向前、y 向左、转角逆时针为正。
/// 使用显式 rear_turn，而不复用 vy，避免“角度”和“速度”在适配层混淆。
struct Control
{
  double vx{0.0};         ///< 本体纵向速度 m/s，不是车轮滚动速度或合速度。
  double vy{0.0};         ///< 仅全向模型使用的独立横向速度 m/s；其他模型必须为零。
  double turn{0.0};       ///< 阿克曼/四轮转向：前轴等效转角 rad；其他模型：角速度 rad/s。
  double rear_turn{0.0};  ///< 仅四轮转向使用的后轴等效转角 rad；其他模型必须为零。
};

/// 启动时配置。长度 m、时间 s、角度 rad；具体 ROS 参数名和调参建议见 config/mpc.yaml。
/// 阿克曼参考点在后轴中心；四轮转向参考点可由前后轴距离定义，默认在两轴中点。
struct Config
{
  Chassis chassis{Chassis::Ackermann};
  std::size_t horizon{20};         ///< 预测控制序列长度；预测时长约为 horizon * dt。
  std::size_t max_iterations{60};  ///< 每次调用的优化迭代上限，不代表一定收敛。
  double dt{0.1};                 ///< 预测积分步长，也必须是应用层的控制调用周期。
  double wheelbase{0.65};
  double front_axle_distance{0.325};  ///< 四轮转向参考点到前轴的正距离；轴距为前后距离之和。
  double rear_axle_distance{0.325};   ///< 四轮转向参考点到后轴的正距离，不使用 wheelbase。
  double target_speed{0.6};       ///< 沿路径参考速度；不是每一步强制执行的实际速度。
  double max_vx{1.0};
  double max_vy{0.6};
  double max_yaw_rate{1.2};
  double max_steering{0.55};
  double max_rear_steering{0.55};     ///< 四轮转向后轴转角上限。
  double max_acceleration{0.8};
  double max_lateral_acceleration{0.8};  ///< 仅全向 vy 变化率，不限制四轮转向向心加速度。
  double max_yaw_acceleration{1.5};
  double max_steering_rate{0.8};
  double max_rear_steering_rate{0.8};  ///< 四轮转向后轴转角变化率 rad/s。
  double position_weight{12.0};
  double heading_weight{3.0};
  double velocity_weight{1.0};
  double effort_weight{0.05};
  double smoothness_weight{0.3};
  double terminal_weight{4.0};
  double gradient_tolerance{1e-4};  ///< 历史字段名；实际是接受步的代价改善阈值，不是梯度。
  double goal_position_tolerance{0.08};
  double goal_yaw_tolerance{0.15};
  double progress_search_distance{2.0};  ///< 路径初始化后每次匹配允许向前搜索的弧长。
};

struct Result
{
  bool valid{false};       ///< false 时禁止执行 command；应用层必须停车并报告 message。
  bool goal_reached{false};
  bool converged{false};   ///< 有限可行解不等于数值收敛；不收敛状态必须向上层暴露。
  std::string message;
  Control command;
  std::vector<Pose> prediction;  ///< 当前位姿 + horizon 个预测位姿；到达时仅含当前位姿。
  double cost{0.0};
  std::size_t iterations{0};
};

double normalize_angle(double angle);
/// 四轮转向的横向速度由两轴转角推导，而不是 Control.vy。
double lateral_velocity(const Control & control, const Config & config);
double yaw_rate(const Control & control, const Config & config);
/// 一个 dt 内恒定本体速度的解析 SE(2) 积分，低角速度时连续退化为直线。
Pose integrate(const Pose & pose, const Control & control, const Config & config);
void validate_config(const Config & config);

class Controller
{
public:
  explicit Controller(Config config);
  /// 非空路径；重复位置合并并保留后一个 yaw。校验失败抛异常，不改变旧路径。
  void set_path(const std::vector<Pose> & path);
  void clear_path();
  bool has_path() const;
  /// 每 dt 调用一次；previous 必须是上次实际发送且在约束内的命令。
  /// 实例会保存路径进度/热启动，不可被多个线程同时调用。
  Result compute(const Pose & pose, const Control & previous);
  const Config & config() const {return config_;}

private:
  struct Reference
  {
    Pose pose;
    Control control;
  };

  double update_progress(const Pose & pose);  ///< 投影到候选线段，更新单调前进的弧长。
  Pose sample(double station) const;         ///< 按弧长插值位置并按模型选择航向语义。
  std::vector<Reference> references(double progress) const;  ///< 预测窗口中的位置/控制参考。
  Control constrain(Control control, const Control & previous) const;
  std::vector<Control> project(
    const std::vector<Control> & controls, const Control & previous) const;
  double evaluate(
    const Pose & pose, const Control & previous, const std::vector<Control> & controls,
    const std::vector<Reference> & refs, std::vector<Pose> * prediction = nullptr) const;

  Config config_;
  std::vector<Pose> path_;
  std::vector<double> stations_;
  std::vector<Control> warm_start_;  ///< 上次整条解序列；下次左移一格复用，不是已发送命令。
  double progress_{0.0};
  bool progress_initialized_{false};
};

}  // namespace path_follow_mpc
