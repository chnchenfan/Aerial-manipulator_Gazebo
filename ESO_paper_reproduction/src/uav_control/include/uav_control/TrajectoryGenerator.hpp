#ifndef TRAJECTORY_GENERATOR_HPP
#define TRAJECTORY_GENERATOR_HPP

#include <Eigen/Dense>
#include <cmath>

// --- 1. 定义飞行模式 ---
enum class TrajectoryMode {
    MODE1_HOVER,        // 模式1: 垂直上升到固定高度 -> 悬停
    MODE2_CIRCLE,       // 模式2: 垂直上升 -> 平滑切入 -> 画圆
    MODE3_SPIRAL,       // 模式3: 螺旋上升 (一边画圈一边上升) -> 到顶悬停
    MODE4_ATTITUDE_SINE // 模式4: [新增] 姿态环专用测试 (定点悬停 + Roll轴正弦摆动)
};

// --- 2. 定义参数结构体 ---
struct TrajectoryParameters {
    // [通用参数]
    double target_height       = 1.5;  // 目标高度 (米)
    double duration            = 10.0; // 动作总耗时 (秒)

    // [模式 2 & 3 专用]
    double circle_radius       = 0.8;  // 圆半径 (米)

    // [模式 2 专用]
    double circle_speed        = 0.5;  // 画圆线速度 (m/s)

    // [模式 3 专用]
    double spiral_turns        = 2.0;  // 上升过程中转几圈

    // [模式 4 姿态测试专用]
    double att_test_amp        = 0.26; // 振幅 (弧度), 0.26rad 约等于 15度
    double att_test_freq       = 0.5;  // 频率 (Hz), 0.5Hz 表示 2秒一个周期
};

// --- 3. 生成器类定义 ---
class TrajectoryGenerator {
public:
    // 构造函数
    TrajectoryGenerator(const TrajectoryParameters& params);

    /**
     * @brief 位置环参考计算 (Pos/Vel/Acc)
     * 适用于 Mode 1, 2, 3。
     * 对于 Mode 4，将返回固定悬停位置 (便于姿态测试时不乱飘)。
     */
    void update(double t, TrajectoryMode mode,
                Eigen::Vector3d& pos_ref,
                Eigen::Vector3d& vel_ref,
                Eigen::Vector3d& acc_ref);

    /**
     * @brief [新增] 姿态环参考计算 (Quat/Omega/Alpha)
     * 专门用于 Mode 4 的姿态测试，生成解析的角速度和角加速度。
     * 对于 Mode 1, 2, 3，默认返回平飞姿态 (Identity) 和零角速度。
     * * @param t             当前时间
     * @param mode          轨迹模式
     * @param q_ref         [输出] 期望四元数
     * @param omega_ref     [输出] 期望角速度 (Body Frame)
     * @param alpha_ref     [输出] 期望角加速度 (Body Frame)
     */
    void updateAttitude(double t, TrajectoryMode mode,
                        Eigen::Quaterniond& q_ref,
                        Eigen::Vector3d& omega_ref,
                        Eigen::Vector3d& alpha_ref);

private:
    TrajectoryParameters params_;

    // 内部辅助: 计算平滑的 S-Curve
    Eigen::Vector3d computeSmoothScalar(double t, double total_time, double target_val);
};

#endif // TRAJECTORY_GENERATOR_HPP
