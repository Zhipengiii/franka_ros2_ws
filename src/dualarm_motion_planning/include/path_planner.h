#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>
#include <vector>
#include <string>
#include <fstream>
#include <geometry_msgs/msg/pose.hpp>

class PathPlanner
{
public:
    // 构造函数，初始化 MoveGroupInterface 和其他参数
    PathPlanner(rclcpp::Node::SharedPtr node, const std::string& group_name);

    // 规划关节空间路径， 输入参数 ： 规划组名称， 目标关节角度
    moveit::planning_interface::MoveGroupInterface::Plan planJointSpacePath(const std::vector<double>& target_joint_values, 
                                                                            const std::vector<double>& current_joint_values);

    // 规划笛卡尔空间路径， 输入参数 ： 规划组名称， 目标位姿
    moveit::planning_interface::MoveGroupInterface::Plan planCartesianSpacePath(const geometry_msgs::msg::Pose& pose_target);

    // 保存路径点到文件， 输入参数 ： 路径， 文件路径
    void saveTrajectoryToFile(const moveit::planning_interface::MoveGroupInterface::Plan& plan, const std::string& file_path);

    // 设置目标关节角度
    void setJointValueTarget(const std::vector<double>& joint_values);

    // 添加冲突点构型的障碍物（在冲突构型的连杆位置添加虚拟障碍）
    void addConflictObstacle(
        const std::vector<double>& conflict_joint_values,  // 冲突点的关节值
        const std::vector<std::string>& link_names,        // 需要添加障碍的连杆名称
        double length, double width, double height         // 障碍物的尺寸             
    );

    // 清除所有通过addConflictObstacle添加的障碍物
    void clearConflictObstacles();

private:
    rclcpp::Node::SharedPtr node_; // 保存节点指针用于日志打印
    moveit::planning_interface::PlanningSceneInterface planning_scene_;  // 规划场景接口
    moveit::planning_interface::MoveGroupInterface move_group_;  // 规划组接口

    // 用于跟踪冲突点障碍物ID，方便后续清除
    std::vector<std::string> conflict_obstacle_ids_;
    
    // 机器人模型和状态（用于计算冲突构型下的连杆位姿）
    moveit::core::RobotModelConstPtr kinematic_model_;
    moveit::core::RobotStatePtr kinematic_state_;
    const moveit::core::JointModelGroup* joint_model_group_;
};

#endif // PATH_PLANNER_H