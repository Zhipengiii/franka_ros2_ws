#include "path_planner.h"

// 构造函数
PathPlanner::PathPlanner(rclcpp::Node::SharedPtr node, const std::string& group_name)
    : node_(node),
      move_group_(node, group_name)
{
    kinematic_model_ = move_group_.getRobotModel();
    kinematic_state_ = std::make_shared<moveit::core::RobotState>(kinematic_model_);

    // --- 核心优化 1: 算法与时限配置 ---
    // 强制指定使用针对窄缝优化的 BiEST 算法 (对应 ompl_planning.yaml 中的配置)
    move_group_.setPlannerId("BiESTkConfigDefault"); 
    
    // 增加单次规划的最大允许时间 (秒)
    move_group_.setPlanningTime(15.0);        
    
    // 增加尝试次数，随机化采样算法在尝试次数越多时成功率越高
    move_group_.setNumPlanningAttempts(20);   

    // --- 核心优化 2: 容差微调 ---
    // 适当放大目标容差（弧度/米），有助于在受限空间内更快找到可行解
    move_group_.setGoalJointTolerance(0.001);
    move_group_.setGoalPositionTolerance(0.001);
    move_group_.setGoalOrientationTolerance(0.01);

    joint_model_group_ = kinematic_model_->getJointModelGroup(group_name);
    kinematic_state_->setToDefaultValues();
    
    RCLCPP_INFO(node_->get_logger(), "已为规划组 [%s] 初始化路径规划器", group_name.c_str());
}

// 规划关节空间路径
moveit::planning_interface::MoveGroupInterface::Plan PathPlanner::planJointSpacePath(const std::vector<double>& target_joint_values, 
                                                                        const std::vector<double>& current_joint_values)
{
    // 设置规划起始点为机械臂当前实际位置
    move_group_.setStartStateToCurrentState();
    
    moveit::core::RobotState start_state(*move_group_.getCurrentState());
    start_state.setJointGroupPositions(joint_model_group_, current_joint_values);
    move_group_.setStartState(start_state);

    // 设置目标位置
    move_group_.setJointValueTarget(target_joint_values);

    // 执行规划
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::core::MoveItErrorCode code = move_group_.plan(plan);
    bool success = (code == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
        RCLCPP_INFO(node_->get_logger(), "路径规划成功! 轨迹点数: %zu", plan.trajectory_.joint_trajectory.points.size());
    } else {
        RCLCPP_ERROR(node_->get_logger(), "路径规划失败! 错误码: %d. 请确认是否存在物理死锁或碰撞步长过大。", code.val);
    }

    return plan;
}

// 规划笛卡尔空间路径
moveit::planning_interface::MoveGroupInterface::Plan PathPlanner::planCartesianSpacePath(const geometry_msgs::msg::Pose& pose_target)
{
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(pose_target);

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.0; // 跳跃阈值
    const double eef_step = 0.01;      // 终端步长 1cm

    // 计算笛卡尔路径
    double fraction = move_group_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    RCLCPP_INFO(node_->get_logger(), "Cartesian path coverage: %.2f%%", fraction * 100.0);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;

    return plan;
}

// 保存路径点到文件
void PathPlanner::saveTrajectoryToFile(const moveit::planning_interface::MoveGroupInterface::Plan& plan, const std::string& file_path)
{
    std::ofstream outfile(file_path);
    if (!outfile.is_open())
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to open file: %s", file_path.c_str());
        return;
    }

    // 遍历路径点并保存
    for (size_t i = 0; i < plan.trajectory_.joint_trajectory.points.size(); ++i)
    {
        const auto& point = plan.trajectory_.joint_trajectory.points[i];
        for (size_t j = 0; j < point.positions.size(); ++j)
        {
            outfile << point.positions[j] << " ";
        }
        outfile << std::endl;
    }

    outfile.close();
    RCLCPP_INFO(node_->get_logger(), "Trajectory saved to: %s", file_path.c_str());
}

// 设置目标关节角度
void PathPlanner::setJointValueTarget(const std::vector<double>& joint_values)
{
    move_group_.setJointValueTarget(joint_values);
}

// 添加冲突点构型的障碍物
void PathPlanner::addConflictObstacle(
    const std::vector<double>& conflict_joint_values, 
    const std::vector<std::string>& link_names, 
    double length, double width, double height)
{
    // 将运动学状态设置为冲突点的构型
    kinematic_state_->setJointGroupPositions(joint_model_group_, conflict_joint_values);
    
    // 更新连杆位姿（正运动学）
    kinematic_state_->update();

    for (const auto& link_name : link_names)
    {
        // 获取连杆当前的全局位姿
        const Eigen::Isometry3d& link_pose = kinematic_state_->getGlobalLinkTransform(link_name);
        
        std::string obstacle_id = "conflict_obstacle_" + link_name;
        
        moveit_msgs::msg::CollisionObject collision_obj;
        collision_obj.header.frame_id = move_group_.getPlanningFrame();
        collision_obj.id = obstacle_id;
        collision_obj.operation = collision_obj.ADD;

        // 定义障碍物形状（盒子）
        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions = {length, width, height};

        // 设置障碍物位姿（与连杆位置重合）
        geometry_msgs::msg::Pose obstacle_pose;
        obstacle_pose.position.x = link_pose.translation().x();
        obstacle_pose.position.y = link_pose.translation().y();
        obstacle_pose.position.z = link_pose.translation().z();
        
        // 设置障碍物朝向（与连杆一致）
        Eigen::Quaterniond quat(link_pose.rotation());
        obstacle_pose.orientation.x = quat.x();
        obstacle_pose.orientation.y = quat.y();
        obstacle_pose.orientation.z = quat.z();
        obstacle_pose.orientation.w = quat.w();

        // 添加到碰撞对象并应用到规划场景
        collision_obj.primitives.push_back(primitive);
        collision_obj.primitive_poses.push_back(obstacle_pose);
        planning_scene_.applyCollisionObject(collision_obj);

        // 记录冲突障碍物ID
        conflict_obstacle_ids_.push_back(obstacle_id);
        
        RCLCPP_INFO(node_->get_logger(), "Added conflict obstacle '%s' at (%.2f, %.2f, %.2f)", 
                obstacle_id.c_str(), 
                obstacle_pose.position.x, 
                obstacle_pose.position.y, 
                obstacle_pose.position.z);
    }
}

// 清除所有冲突点障碍物
void PathPlanner::clearConflictObstacles()
{
    if (conflict_obstacle_ids_.empty()) 
    {
        RCLCPP_INFO(node_->get_logger(), "No conflict obstacles to clear.");
        return;
    }

    // 移除所有记录的冲突障碍物
    std::vector<std::string> object_ids = conflict_obstacle_ids_;
    planning_scene_.removeCollisionObjects(object_ids);
    
    RCLCPP_INFO(node_->get_logger(), "Cleared %zu conflict obstacles.", conflict_obstacle_ids_.size());
    
    // 清空ID列表
    conflict_obstacle_ids_.clear();
}
