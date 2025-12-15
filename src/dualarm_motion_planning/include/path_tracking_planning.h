#ifndef PATH_TRACKING_PLANNING_H
#define PATH_TRACKING_PLANNING_H

#include <vector>
#include <array>
#include <string>
#include <cmath>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <iomanip>
#include <limits>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

// 关节空间运动生成类声明
class PathTrackingPlanning
{
public:
    using Vector7d = Eigen::Matrix<double, 7, 1>;

    struct TrajectoryPoint
    {
        double position[7]; // 机械臂7个关节的关节空间构型（弧度）
        double velocity[7]; // 机械臂7个关节的关节速度（弧度/秒）
    };

    struct Trajectory
    {
        std::vector<TrajectoryPoint> points; // 包含若干个轨迹点
    };
    Trajectory trajectory;

    // 构造函数
    PathTrackingPlanning(double v_actual_max, double a_actual_max, Vector7d v_0_, Eigen::MatrixXd q_path_);
    
    // 运动轨迹生成函数
    Trajectory startPlanning(double sample_time);
    
    // 获取规划出的轨迹数据大小
    size_t getTrajectorySize();
    
    // 画图函数
    void plotTrajectory(const Trajectory& trajectory, const std::string& data_dir, const std::string& file_name);
    
    // 保存轨迹数据函数
    void saveTrajectoryData(std::string position_file_path, std::string velocity_file_path);

private:
    // 对每个相邻路径点段：样条段参数结构体 s(t) = a + b*dt + c*dt^2 + d*dt^3
    struct SplineSegment {
        double a, b, c, d, x; // x是该段的起始时间
    };

    // 成员变量
    Vector7d v_max_; // 最大速度限制
    Vector7d a_max_; // 最大加速度限制
    Vector7d v_0_;   // 初始速度
    
    // 存储处理后的路径点
    std::vector<Vector7d> waypoints_;
    
    // 存储计算好的时间节点
    std::vector<double> time_knots_;
    
    // 存储7个关节的所有样条段
    std::vector<std::vector<SplineSegment>> joint_splines_;

    // 内部初始化函数
    void initializeMotion(const Eigen::MatrixXd& q_path);
    
    // 内部函数：初始化轨迹容器
    void initializeTrajectory();

    // 核心算法：计算三次样条系数
    std::vector<SplineSegment> computeSpline(const std::vector<double>& times, const std::vector<double>& positions);
    
    // 核心算法：对样条进行采样
    void sampleSpline(double t, const std::vector<SplineSegment>& spline, double& pos, double& vel);
};

#endif // PATH_TRACKING_PLANNING_H
