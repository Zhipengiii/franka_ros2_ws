#ifndef PATH_TRACKING_PLANNING_H
#define PATH_TRACKING_PLANNING_H

#include <algorithm>
#include <array>
#include <cmath>
#include <eigen3/Eigen/Core>
#include <sstream>
#include <iostream>
#include <fstream>
#include <chrono>
#include <vector>
#include <sensor_msgs/msg/joint_state.hpp>
#include "rclcpp/rclcpp.hpp"

// 关节空间运动生成类声明
class PathTrackingPlanning
{
public:
    using Vector7d = Eigen::Matrix<double, 7, 1, Eigen::ColMajor>;
    using Vector7i = Eigen::Matrix<int, 7, 1, Eigen::ColMajor>;

    Vector7d q_current; // 当前位置
    Vector7d v_current; // 当前速度

    Vector7d v_0; // 初始速度
    Vector7d q_0; // 初始位置
    Vector7d q_f; // 最终位置
    Vector7d q_via; // 路径点位置
    int i_via = 0; // 路径点索引
    size_t n_via; // 路径点数量
    size_t n_dof = 7; // 机械臂自由度
    double sample_time_; // 采样时间

    double t_f_sync; // 总运动同步时间
    bool motion_finished = false;

    // 存储关节空间初始构型
    std::array<double, 7> init_position = {0, 0, 0, 0, 0, 0, 0};

    // 定义轨迹点结构体
    struct TrajectoryPoint
    {
        double position[7]; // 机械臂7个关节的关节空间构型（弧度）
        double velocity[7]; // 机械臂7个关节的关节速度（弧度/秒）
    };

    // 定义轨迹结构体
    struct Trajectory
    {
        std::vector<TrajectoryPoint> points; // 包含若干个轨迹点
    };

    // 初始化一个轨迹结构体
    Trajectory trajectory;

    PathTrackingPlanning(double v_actual_max, double a_actual_max, Vector7d v_0_, Eigen::MatrixXd q_path_); // 构造函数声明
    void calculateChangeDirection(); // 计算方向转换点函数
    void initializeMotion(Vector7d v_0_, Eigen::MatrixXd q_path_); // 运动初始化
    void updateParameters(); // 更新轨迹参数函数声明
    void calculateTminTraj(); // 计算时间最优轨迹函数声明
    void calculateViaTimeANDdetermineSyncTime(); // 计算到路径点的时间并确定同步运动时间函数声明
    void adjustViaTVP(); // 调整路径点轨迹参数
    void calculateViaTraj(); // 计算到路径点轨迹函数声明
    void calculateDesiredValues(double t); // 根据轨迹参数计算轨迹值函数声明
    void initializeTrajectory(); // 初始化轨迹结构体
    size_t getTrajectorySize(); // 获取规划出的轨迹数据大小
    PathTrackingPlanning::Trajectory startPlanning(double sample_time); // 运动轨迹生成函数
    void plotTrajectory(const Trajectory& trajectory, const std::string& data_dir, const std::string& file_name); // 画图函数
    void saveTrajectoryData(std::string position_file_path, std::string velocity_file_path); // 保存轨迹数据函数

private:
    Eigen::MatrixXd q_path; // 存储输入的路径数据(n_via, n_dof)
    Eigen::VectorXi n_change_direction;   // 每个关节改变方向的次数 
    Eigen::MatrixXi i_via_change_direction;  // 方向改变路径点的索引
    Eigen::VectorXi i_change_direction;  

    Vector7d q_delta;
    Vector7i q_delta_sign;
    Vector7d q_via_delta;
    Vector7d v0; // 计算轨迹时的绝对速度

    // 梯形速度曲线参数
    Vector7d acc;
    Vector7d vm;
    Vector7d t1;
    Vector7d t2;
    Vector7d tf;
    double t_via_sync;

    Vector7d v_max; // 实际最大速度
    Vector7d a_max; // 实际最大加速度

    double time = 0.0;
};

// 转向判断函数
inline int determineSign(double value) 
{
    return (value > 0) ? 1 : ((value < 0) ? -1 : 0);
}

#endif // PATH_TRACKING_PLANNING_H

