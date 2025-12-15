#ifndef DUAL_ARM_COORDINATOR_H
#define DUAL_ARM_COORDINATOR_H

#include <Eigen/Core>
#include <vector>
#include <array>
#include <string>
#include <fstream>
#include <iostream>
#include <cmath>
#include "collision_detection.h"  
#include "path_planner.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include "path_tracking_planning.h"

class DualArmCoordinator 
{
public:
    std::string package_path; // 存储生成的轨迹数据文件的相对路径
    bool deadlock_flag = false; // 阻塞状态标志
    bool diag_collision_flag = false; // 对角线碰撞标志
    bool prior_r1_ = false; // 协调因子和优先级标志
    bool prior_r2_ = false;
    double delay_time_r1_ = 0.0;
    double delay_time_r2_ = 0.0;
    moveit::planning_interface::MoveGroupInterface::Plan plan; // 路径重规划结果

    // 定义导致阻塞的冲突机器人
    enum BlockingConflictR
    {
        CONFLICT_R1,      // R1 为冲突机器人
        CONFLICT_R2,      // R2 为冲突机器人
        CONFLICT_R1_R2    // R1和R2 都为冲突机器人
    };
    BlockingConflictR blocking_conflict_r; // 阻塞优先级

    // 构造函数
    DualArmCoordinator(rclcpp::Node::SharedPtr node,
                        double collision_threshold, int sample_size_max, double sample_time,
                        const std::vector<std::string>& r1_links_in,
                        const std::vector<std::string>& r2_links_in ,
                        const std::string& r1_group_name_in,
                        const std::string& r2_group_name_in);

    // 核心协调算法入口
    void coordinate(const Eigen::MatrixXd& cartesian_r1, const Eigen::MatrixXd& cartesian_r2,
                    const PathTrackingPlanning::Trajectory& trajectory_r1,
                    const PathTrackingPlanning::Trajectory& trajectory_r2);

    void saveCollisionMatrix(const std::string& file_path); // 保存碰撞矩阵
    std::pair<int, int> getMaxCollisionRow(); // 获取最大碰撞数量行,返回值<索引， 碰撞数量>
    std::pair<int, int> getMaxCollisionCol(); // 获取最大碰撞数量列,返回值<索引， 碰撞数量>

private:
    rclcpp::Node::SharedPtr node_;
    
    // 核心参数
    double collision_threshold_;    // 碰撞检测阈值
    int sample_size_max_;           // 最大采样点数
    double sample_time_;            // 采样时间间隔

    const double ARC_LENGTH_THRESHOLD = 0.25; // 标称轨迹弧长（反应机械臂移动距离）(单位：米) -> 碰撞阈值的2.5倍
    // 存储计算出的笛卡尔空间弧长
    double r1_arc_length_ = 0.0;
    double r2_arc_length_ = 0.0;
    
    // 定义机器人包含的连杆名称
    std::vector<std::string> r1_links;
    std::vector<std::string> r2_links;
    // 定义机器人规划组名称
    std::string r1_group_name;
    std::string r2_group_name;

    // 中间变量
    Eigen::MatrixXd collision_matrix_;       // 碰撞矩阵 (sample_r1 x sample_r2)
    Eigen::MatrixXd collision_boundary_r1_;  // 左臂碰撞边界 (3 x sample_r1)
    Eigen::MatrixXd collision_boundary_r2_;  // 右臂碰撞边界 (3 x sample_r2)
    std::array<bool, 4> collision_matrix_vector_; // 碰撞矩阵向量化结果 [碰撞矩阵的上、下、左、右边界向量]

    // 内部方法声明
    void initializeVariables(); // 初始化变量的方法
    void generateCollisionMatrix(const Eigen::MatrixXd& cartesian_r1, const Eigen::MatrixXd& cartesian_r2); // 生成碰撞矩阵函数
    void generateCollisionBoundaries(); // 生成碰撞边界矩阵函数
    void generateCollisionMatrixVector(); // 生成碰撞矩阵向量函数
    void determinePriorityAndDeadlock(); // 确定优先级和阻塞状态函数
    void calculateCoordFactors(); // 计算协调因子函数
    bool isDiagCollision(); // 检查碰撞矩阵对角线是否有碰撞函数
    double calculateCartesianArcLength(const Eigen::MatrixXd& cartesian_data); // 计算笛卡尔轨迹弧长
    void determineBlockingConflictR(); // 确定导致阻塞的冲突机器人
    bool blockingReplanning(const PathTrackingPlanning::Trajectory& trajectory_r1, 
                            const PathTrackingPlanning::Trajectory& trajectory_r2); // 阻塞重规划函数
    
};

#endif // DUAL_ARM_COORDINATOR_H