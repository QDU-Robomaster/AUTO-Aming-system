#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <functional>
#include <limits>
#include <utility>
#include <vector>

#include "armor.hpp"
#include "libxr.hpp"

/**
 * @brief Solve projectile pitch/yaw aiming for rotating multi-armor targets.
 *
 * 提供单方向空气阻力模型的弹道解算（pitch），以及若干开火逻辑与装甲板选择逻辑。
 */
class SolveTrajectory
{
 public:
  /// 弹道常量
  static constexpr float PI = 3.1415926535f;
  static constexpr float GRAVITY = 9.78f;

  /// 目标装甲板类型
  enum TargetArmorId
  {
    ARMOR_OUTPOST = 0,
    ARMOR_HERO = 1,
    ARMOR_ENGINEER = 2,
    ARMOR_INFANTRY3 = 3,
    ARMOR_INFANTRY4 = 4,
    ARMOR_INFANTRY5 = 5,
    ARMOR_GUARD = 6,
    ARMOR_BASE = 7
  };

  /// 不同目标装甲板数量
  enum TargetArmorNum
  {
    ARMOR_NUM_OUTPOST = 3,  ///< 前哨站三块
    ARMOR_NUM_NORMAL = 4    ///< 普通目标四块
  };

  /// 弹丸类型（保留枚举）
  enum BulletType
  {
    BULLET_17 = 0,
    BULLET_42 = 1
  };

  /**
   * @brief 单块装甲板在世界坐标系的位姿简化量
   */
  struct TargetPostion
  {
    float x;    ///< world-x [m]
    float y;    ///< world-y [m]
    float z;    ///< world-z [m]
    float yaw;  ///< armor yaw in world frame [rad]
  };

  /**
   * @brief 追踪到的目标（机器人）聚合信息
   */
  struct Target
  {
    bool tracking;                         ///< 是否正在追踪
    ArmorNumber id;                        ///< 目标ID（来自 armor.hpp）
    int armors_num;                        ///< 装甲板数量（3 或 4）
    Eigen::Matrix<double, 3, 1> position;  ///< 世界系目标中心位置 [m]
    Eigen::Matrix<double, 3, 1> velocity;  ///< 世界系目标线速度 [m/s]
    double yaw;                            ///< 目标当前 yaw [rad]
    double v_yaw;                          ///< 目标 yaw 角速度 [rad/s]
    double radius_1;                       ///< 旋转半径1 [m]
    double radius_2;                       ///< 旋转半径2 [m]
    double dz;                             ///< 竖直偏差（保留）
  };

  /**
   * @brief 构造函数
   * @param k 空气阻力系数（简化模型）
   * @param bias_time 下发到开火的系统延迟（ms）
   * @param s_bias 枪口前推距离偏置 [m]
   * @param z_bias yaw轴电机到枪口水平面的垂直偏置 [m]
   */
  SolveTrajectory(const float& k, const int& bias_time, const float& s_bias,
                  const float& z_bias);

  /**
   * @brief 初始化弹速
   * @param velocity 当前弹速 [m/s]（NaN 则取默认 18）
   */
  void Init(double velocity);

  /**
   * @brief 单方向空气阻力模型（解算给定角度下的弹道高程）
   * @param s 水平距离 [m]
   * @param v 弹速 [m/s]
   * @param angle 发射仰角 [rad]
   * @return 弹着点高度（相对发射点）z [m]
   * @note 同时会更新 @ref fly_time
   */
  float MonoDirectionalAirResistanceModel(float s, float v, float angle);

  /**
   * @brief Pitch 角弹道补偿（迭代求解发射仰角）
   * @param s 目标水平距离 [m]
   * @param z 目标相对高度 [m]
   * @param v 弹速 [m/s]
   * @return pitch 仰角 [rad]
   */
  float PitchTrajectoryCompensation(float s, float z, float v);

  /**
   * @brief 线性预测判断是否开火
   * @param tmp_yaw 候选装甲板 yaw（当前帧）
   * @param v_yaw 目标 yaw 角速度 [rad/s]
   * @param timeDelay 延迟（通信 + 弹道飞行）[s]
   * @return 是否满足开火条件
   */
  bool ShouldFire(float tmp_yaw, float v_yaw, float timeDelay);

  /// 回调：是否开火（由 fireLogic 触发）
  using FireCallback = std::function<void(bool)>;

  /// 设置开火回调
  void SetFireCallback(FireCallback callback) { fire_callback_ = std::move(callback); }

  /**
   * @brief 根据当前观测，推算各装甲板在世界系的位置
   * @param msg 目标状态
   * @param use_1 true 使用 radius_1，否则 radius_2（两者交替使用以遍历四块）
   * @param use_average_radius 若为 true，则使用 (r1+r2)/2
   * @note 结果保存在 @ref tar_position 与 @ref tmp_yaws
   */
  void CalculateArmorPosition(Target* msg, bool use_1, bool use_average_radius);

  /**
   * @brief 计算 pitch 与 yaw
   * @param idx 被选中的装甲板索引
   * @param msg 目标状态
   * @param timeDelay 时延（通信+弹道）[s]
   * @param s_bias 枪口前推偏置 [m]
   * @param z_bias 枪口垂直偏置 [m]
   * @param current_v 当前弹速 [m/s]
   * @param use_target_center_for_yaw yaw 计算是否使用目标中心（否则用预测的装甲板点）
   * @param[out] aim_x 预测打击点 x
   * @param[out] aim_y 预测打击点 y
   * @param[out] aim_z 预测打击点 z
   * @return <pitch, yaw> [rad]
   */
  std::pair<float, float> CalculatePitchAndYaw(int idx, Target* msg, float timeDelay,
                                               float s_bias, float z_bias,
                                               float current_v,
                                               bool use_target_center_for_yaw,
                                               float& aim_x, float& aim_y, float& aim_z);

  /**
   * @brief 装甲板选择策略
   * @param msg 目标状态
   * @param select_by_min_yaw 若为 true，选择与当前枪口 yaw
   * 差最小者；否则选择欧氏距离最近者
   * @return 选中的装甲板索引（0 ~ armors_num-1）
   */
  int SelectArmor(Target* msg, bool select_by_min_yaw);

  /**
   * @brief 优先级较高的开火逻辑（“顶部优先”）
   * @param[out] pitch 结果 pitch [rad]
   * @param[out] yaw 结果 yaw [rad]
   * @param[out] aim_x 预测打击点 x
   * @param[out] aim_y 预测打击点 y
   * @param[out] aim_z 预测打击点 z
   * @param msg 目标状态
   */
  void FireLogicIsTop(float& pitch, float& yaw, float& aim_x, float& aim_y, float& aim_z,
                      Target* msg);

  /**
   * @brief 默认开火逻辑（距离最近优先）
   * @param[out] pitch 结果 pitch [rad]
   * @param[out] yaw 结果 yaw [rad]
   * @param[out] aim_x 预测打击点 x
   * @param[out] aim_y 预测打击点 y
   * @param[out] aim_z 预测打击点 z
   * @param msg 目标状态
   */
  void FireLogicDefault(float& pitch, float& yaw, float& aim_x, float& aim_y,
                        float& aim_z, Target* msg);

  /**
   * @brief 自动解算弹道（根据最优决策流转到具体逻辑）
   * @param[out] pitch pitch [rad]
   * @param[out] yaw yaw [rad]
   * @param[out] aim_x 打击点 x
   * @param[out] aim_y 打击点 y
   * @param[out] aim_z 打击点 z
   * @param msg 目标状态
   */
  void AutoSolveTrajectory(float& pitch, float& yaw, float& aim_x, float& aim_y,
                           float& aim_z, Target* msg);

 private:
  float k_;            ///< 空气阻力系数
  float current_v_{};  ///< 当前弹速 [m/s]
  double fly_time_{};  ///< 最近一次模型计算得到的飞行时间 [s]

  int bias_time_;  ///< 系统时延 [ms]
  float s_bias_;   ///< 枪口前推 [m]
  float z_bias_;   ///< 枪口垂直偏置 [m]

  float tar_yaw_{};  ///< 当前目标 yaw（供内部计算）

  TargetPostion tar_position_[4]{};  ///< 记录 3/4 块装甲板的位置
  std::vector<float> tmp_yaws_;      ///< 对应各装甲板的 yaw（一个周期内）

  float min_yaw_in_cycle_{std::numeric_limits<float>::max()};
  float max_yaw_in_cycle_{std::numeric_limits<float>::lowest()};

  /**
   * @brief 完整气动模型（预留，尚未实现）
   * @param s 水平距离 [m]
   * @param v 弹速 [m/s]
   * @param angle 发射仰角 [rad]
   * @return 弹着点高度 z [m]
   */
  float CompleteAirResistanceModel(float s, float v, float angle);

  FireCallback fire_callback_;  ///< 开火回调
};
