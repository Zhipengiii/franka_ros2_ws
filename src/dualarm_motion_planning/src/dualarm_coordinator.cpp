#include "dualarm_coordinator.h"

// 构造函数
DualArmCoordinator::DualArmCoordinator(rclcpp::Node::SharedPtr node,
                    double collision_threshold, int sample_size_max, double sample_time,
                    const std::vector<std::string>& r1_links_in,
                    const std::vector<std::string>& r2_links_in ,
                    const std::string& r1_group_name_in,
                    const std::string& r2_group_name_in)
    : node_(node), 
      collision_threshold_(collision_threshold),
      sample_size_max_(sample_size_max),
      sample_time_(sample_time),
      r1_links(r1_links_in),
      r2_links(r2_links_in),
      r1_group_name(r1_group_name_in),
      r2_group_name(r2_group_name_in)
{
#ifdef PACKAGE_SOURCE_DIR
    package_path = PACKAGE_SOURCE_DIR;
#else
    // 如果未定义宏，则回退到 install 路径或当前路径
    try {
        package_path = ament_index_cpp::get_package_share_directory("dualarm_motion_planning");
    } catch (...) {
        package_path = ".";
    }
#endif
}

void DualArmCoordinator::initializeVariables() 
{
    deadlock_flag = false; // 死锁标志
    diag_collision_flag = false; // 对角线碰撞标志
    prior_r1_ = false; // 左臂优先级标志
    prior_r2_ = false; // 右臂优先级标志
    delay_time_r1_ = 0.0; // 左臂延时时间
    delay_time_r2_ = 0.0; // 右臂延时时间
    collision_matrix_.resize(0, 0); // 碰撞矩阵初始化为空
    collision_boundary_r1_.resize(0, 0); // 左臂碰撞边界矩阵初始化为空
    collision_boundary_r2_.resize(0, 0); // 右臂碰撞边界矩阵初始化为空
    collision_matrix_vector_.fill(false); // 碰撞矩阵向量化结果初始化为false
    plan.trajectory_.joint_trajectory.points.clear(); // 清空路径重规划结果
}

void DualArmCoordinator::generateCollisionMatrix(const Eigen::MatrixXd& cartesian_r1, const Eigen::MatrixXd& cartesian_r2) 
{
    int sample_r1 = cartesian_r1.cols();
    int sample_r2 = cartesian_r2.cols();
    collision_matrix_.resize(sample_r1, sample_r2);

    std::array<double, 3> s1_v1, s1_v2, s2_v1, s2_v2; // 定义碰撞检测线段
    // 遍历每一对样本，检测碰撞
    for (int i = 0; i < sample_r1; ++i) 
    {
        for (int j = 0; j < sample_r2; ++j) 
        {
            bool flag_collision = false;
            // 遍历部分可能存在碰撞的关节对（提高构建碰撞矩阵的效率）
            for (size_t a = 0; a < 3; ++a) 
            {
                for (size_t b = 0; b < 3; ++b) 
                {
                    // 左臂关节a的线段端点（link a到link a+1）
                    s1_v1 = {
                        cartesian_r1(3*(7-a)+0, i),
                        cartesian_r1(3*(7-a)+1, i),
                        cartesian_r1(3*(7-a)+2, i)
                    };
                    s1_v2 = {
                        cartesian_r1(3*(6-a)+0, i),
                        cartesian_r1(3*(6-a)+1, i),
                        cartesian_r1(3*(6-a)+2, i)
                    };
                    // 右臂关节b的线段端点（link b到link b+1）
                    s2_v1 = {
                        cartesian_r2(3*(7-b)+0, j),
                        cartesian_r2(3*(7-b)+1, j),
                        cartesian_r2(3*(7-b)+2, j)
                    };
                    s2_v2 = {
                        cartesian_r2(3*(6-b)+0, j),
                        cartesian_r2(3*(6-b)+1, j),
                        cartesian_r2(3*(6-b)+2, j)
                    };
                    // 调用碰撞检测函数
                    if (collision_detection(s1_v1, s1_v2, s2_v1, s2_v2, collision_threshold_)) 
                    {
                        flag_collision = true;
                        break;  // 只要一对关节碰撞，即标记为碰撞
                    }
                }
                if (flag_collision) break;
            }
            collision_matrix_(i, j) = flag_collision ? 1 : 0;
        }
    }
}

void DualArmCoordinator::generateCollisionBoundaries() 
{
    int sample_r1 = collision_matrix_.rows();
    int sample_r2 = collision_matrix_.cols();

    // 初始化左臂边界矩阵（3行：碰撞标志、下边界、上边界）
    collision_boundary_r1_.resize(3, sample_r1);
    collision_boundary_r1_.setZero();
    collision_boundary_r1_.row(1).setConstant(sample_r2 - 1);  // 下边界初始化为最大索引

    // 遍历每一个样本
    for (int i = 0; i < sample_r1; ++i) 
    {
        for (int j = 0; j < sample_r2; ++j) 
        {
            if (collision_matrix_(i, j)) 
            {
                collision_boundary_r1_(0, i) = 1;  // 标记碰撞发生
                if (j < collision_boundary_r1_(1, i)) 
                {
                    collision_boundary_r1_(1, i) = j;  // 更新下边界（最小碰撞索引）
                }
                if (j > collision_boundary_r1_(2, i)) 
                {
                    collision_boundary_r1_(2, i) = j;  // 更新上边界（最大碰撞索引）
                }
            }
        }
    }

    // 初始化右臂边界矩阵（对称处理）
    collision_boundary_r2_.resize(3, sample_r2);
    collision_boundary_r2_.setZero();
    collision_boundary_r2_.row(1).setConstant(sample_r1 - 1);

    for (int j = 0; j < sample_r2; ++j) 
    {
        for (int i = 0; i < sample_r1; ++i) 
        {
            if (collision_matrix_(i, j)) 
            {
                collision_boundary_r2_(0, j) = 1;
                if (i < collision_boundary_r2_(1, j)) 
                {
                    collision_boundary_r2_(1, j) = i;
                }
                if (i > collision_boundary_r2_(2, j)) 
                {
                    collision_boundary_r2_(2, j) = i;
                }
            }
        }
    }
}

void DualArmCoordinator::generateCollisionMatrixVector()
{
    // 获取双臂样本数量
    int sample_r1 = collision_boundary_r1_.cols();
    int sample_r2 = collision_boundary_r2_.cols();

    // 生成碰撞矩阵向量
    collision_matrix_vector_[0] = collision_boundary_r1_(0, 0);           // 碰撞矩阵的上边界向量
    collision_matrix_vector_[1] = collision_boundary_r1_(0, sample_r1-1); // 碰撞矩阵的下边界向量
    collision_matrix_vector_[2] = collision_boundary_r2_(0, 0);           // 碰撞矩阵的左边界向量
    collision_matrix_vector_[3] = collision_boundary_r2_(0, sample_r2-1); // 碰撞矩阵的右边界向量

    // 打印调试
    std::cout << "Collision Matrix Vector: ["
            << collision_matrix_vector_[0] << ", "
            << collision_matrix_vector_[1] << ", "
            << collision_matrix_vector_[2] << ", "
            << collision_matrix_vector_[3] << "]" << std::endl;
}

void DualArmCoordinator::determinePriorityAndDeadlock() 
{
    prior_r1_ = prior_r2_ = false;
    int sample_r1 = collision_boundary_r1_.cols();
    int sample_r2 = collision_boundary_r2_.cols();

    if (sample_r1 > 0 && sample_r2 > 0) 
    {
        // 左下角可行（R1优先：左臂最后一个样本无碰撞，右臂第一个样本无碰撞）
        if (!collision_matrix_vector_[1] && !collision_matrix_vector_[2]) 
        {
            prior_r1_ = true;
        }
        // 右上角可行（R2优先：右臂最后一个样本无碰撞，左臂第一个样本无碰撞）
        if (!collision_matrix_vector_[0] && !collision_matrix_vector_[3]) 
        {
            prior_r2_ = true;
        }
    }

    deadlock_flag = !prior_r1_ && !prior_r2_;
}


/*

    待优化逻辑，需要判断完整的并发运动轨迹

*/
bool DualArmCoordinator::isDiagCollision() 
{
    int num_diagonal = std::min(collision_matrix_.rows(), collision_matrix_.cols());
    for (int i = 0; i < num_diagonal; ++i) 
    {
        if (collision_matrix_(i, i) == 1) 
        {
            diag_collision_flag = true; // 对角线上有碰撞
            return diag_collision_flag;
        }
    }
    
    diag_collision_flag = false; // 对角线上无碰撞
    return diag_collision_flag; 
}

void DualArmCoordinator::calculateCoordFactors()
{
    int num_diagonal = std::min(collision_matrix_.rows(), collision_matrix_.cols());
    double collision_avoidance_margin = sample_size_max_ * sample_time_; // 碰撞避免的安全边界

    // R1优先，协调R2（寻找最严格的下边界约束）
    if (prior_r1_)
    {
        double max_exceed = 0.0;
        // int critical_i = 0, critical_j = 0;
        for (int i = 0; i < num_diagonal; ++i) 
        {
            int j_lower = collision_boundary_r1_(1, i);
            double exceed = i - j_lower;
            if (exceed > max_exceed) 
            {
                max_exceed = exceed;
                // critical_i = i;
                // critical_j = j_lower;
            }
        }
        // 计算协调因子
        delay_time_r2_ = (max_exceed + collision_avoidance_margin) * sample_time_;
    }

    // R2优先，协调R1（寻找最严格的下边界约束）
    if (prior_r2_) 
    {
        // R2优先，协调R1（对称处理）
        double max_exceed = 0.0;
        // int critical_i = 0, critical_j = 0;
        for (int j = 0; j < num_diagonal; ++j) 
        {
            int i_lower = collision_boundary_r2_(1, j);
            double exceed = j - i_lower;
            if (exceed > max_exceed) 
            {
                max_exceed = exceed;
                // critical_i = i_lower;
                // critical_j = j;
            }
        }
        delay_time_r1_ = (max_exceed + collision_avoidance_margin) * sample_time_;
    }
}

void DualArmCoordinator::determineBlockingConflictR()
{
    if ((!collision_matrix_vector_[0] && !collision_matrix_vector_[1] && collision_matrix_vector_[2] && collision_matrix_vector_[3])      // [0 0 1 1]
        ||(!collision_matrix_vector_[0] && collision_matrix_vector_[1] && collision_matrix_vector_[2] && collision_matrix_vector_[3])     // [0 1 1 1]
        ||(collision_matrix_vector_[0] && !collision_matrix_vector_[1] && collision_matrix_vector_[2] && collision_matrix_vector_[3]))    // [1 0 1 1]
    {
        blocking_conflict_r = CONFLICT_R1; // R1为导致死锁的冲突机器人，对其进行路径重规划
    }
    else if ((collision_matrix_vector_[0] && collision_matrix_vector_[1] && !collision_matrix_vector_[2] && !collision_matrix_vector_[3])      // [1 1 0 0]
            ||(collision_matrix_vector_[0] && collision_matrix_vector_[1] && !collision_matrix_vector_[2] && collision_matrix_vector_[3])      // [1 1 0 1]
            ||(collision_matrix_vector_[0] && collision_matrix_vector_[1] && collision_matrix_vector_[2] && !collision_matrix_vector_[3]))     // [1 1 1 0]
    {
        blocking_conflict_r = CONFLICT_R2; // R2为导致死锁的冲突机器人，对其进行路径重规划
    }
    else if ((!collision_matrix_vector_[0] && collision_matrix_vector_[1] && !collision_matrix_vector_[2] && collision_matrix_vector_[3])      // [0 1 0 1]
            ||(collision_matrix_vector_[0] && !collision_matrix_vector_[1] && collision_matrix_vector_[2] && !collision_matrix_vector_[3])     // [1 0 1 0]
            ||(collision_matrix_vector_[0] && collision_matrix_vector_[1] && collision_matrix_vector_[2] && collision_matrix_vector_[3]))      // [1 1 1 1]
    {
        blocking_conflict_r = CONFLICT_R1_R2; // R1和R2 都为冲突机器人
    }
}

void DualArmCoordinator::blockingReplanning(const PathTrackingPlanning::Trajectory& trajectory_r1,
                                            const PathTrackingPlanning::Trajectory& trajectory_r2)
{
    if (blocking_conflict_r == CONFLICT_R1)
    {
        PathPlanner path_planner_r1(node_, r1_group_name);  // 左臂路径规划器实例化

        std::pair<int, int> max_collision_row = getMaxCollisionRow(); // 获取最大碰撞数量行
        
        // 打印调试
        std::cout << "导致死锁阻塞的冲突机器人为 : R1, 冲突点索引 : " << max_collision_row.first
                << ", 碰撞次数: " << max_collision_row.second << std::endl;

        // 获取 R1 冲突点构型
        std::vector<double> obstacle_r1;
        for (int i = 0; i < 7; ++i) 
        {
            obstacle_r1.push_back(trajectory_r1.points[max_collision_row.first].position[i]);
        }
        // 获取 R1 初始位置 和 目标位置
        std::vector<double> start_r1, end_r1;
        for (int i = 0; i <7; ++i) 
        {
            start_r1.push_back(trajectory_r1.points[0].position[i]);
            end_r1.push_back(trajectory_r1.points[trajectory_r1.points.size() - 1].position[i]);
        }

        /* 设置 R1 冲突点构型为障碍物 */
        // 定义需要添加障碍物的连杆名称(只考虑可能发生碰撞的连杆)
        std::vector<std::string> r1_collision_links = {r1_links[r1_links.size()-2],
                                                        r1_links[r1_links.size()-1]};
        // 添加冲突点障碍物
        path_planner_r1.addConflictObstacle(obstacle_r1, r1_collision_links, 
                                            collision_threshold_, collision_threshold_, collision_threshold_);

        // R1 进行路径重规划，避开自身冲突点构型
        plan = path_planner_r1.planJointSpacePath(end_r1, start_r1);
        if (!plan.trajectory_.joint_trajectory.points.empty()) 
        {
            // 保存右臂重规划路径
            path_planner_r1.saveTrajectoryToFile(plan, package_path + "/data/trajectory_r1_replanning.txt");
            RCLCPP_INFO(node_->get_logger(), "R1 路径重规划成功，已保存到 trajectory_r1_replanning.txt");
        }

        // 清除 R1 冲突点障碍物
        path_planner_r1.clearConflictObstacles();
    }
    else if (blocking_conflict_r == CONFLICT_R2)
    {
        PathPlanner path_planner_r2(node_, r2_group_name); // 右臂路径规划器实例化

        std::pair<int, int> max_collision_col = getMaxCollisionCol(); // 获取最大碰撞数量列
        
        // 打印调试
        std::cout << "导致死锁阻塞的冲突机器人为 : R2, 冲突点索引 : " << max_collision_col.first
                << ", 碰撞次数: " << max_collision_col.second << std::endl;

        // 获取 R2 冲突点构型
        std::vector<double> obstacle_r2;
        for (int i = 0; i < 7; ++i) 
        {
            obstacle_r2.push_back(trajectory_r2.points[max_collision_col.first].position[i]);
        }
        // 获取 R2 初始位置 和 目标位置
        std::vector<double> start_r2, end_r2;
        for (int i = 0; i < 7; ++i) 
        {
            start_r2.push_back(trajectory_r2.points[0].position[i]);
            end_r2.push_back(trajectory_r2.points[trajectory_r2.points.size() - 1].position[i]);
        }

        /* 设置 R2 冲突点构型为障碍物 */
        // 定义需要添加障碍物的连杆名称(只考虑可能发生碰撞的连杆)
        std::vector<std::string> r2_collision_links = {r2_links[r1_links.size()-2],
                                                        r2_links[r1_links.size()-1]};
        // 添加冲突点障碍物
        path_planner_r2.addConflictObstacle(obstacle_r2, r2_collision_links, 
                                            collision_threshold_, collision_threshold_, collision_threshold_);

        // R2 进行路径重规划，避开自身冲突点构型
        plan = path_planner_r2.planJointSpacePath(end_r2, start_r2);
        if (!plan.trajectory_.joint_trajectory.points.empty()) 
        {
            // 保存右臂重规划路径
            path_planner_r2.saveTrajectoryToFile(plan, package_path + "/data/trajectory_r2_replanning.txt");
            RCLCPP_INFO(node_->get_logger(), "R2 路径重规划成功，已保存到 trajectory_r2_replanning.txt");
        }

        // 清除 R2 冲突点障碍物
        path_planner_r2.clearConflictObstacles();
    }
    else if (blocking_conflict_r == CONFLICT_R1_R2)
    {
        // 获取最大碰撞行和列
        std::pair<int, int> max_collision_row = getMaxCollisionRow(); // <行索引, 碰撞次数>
        std::pair<int, int> max_collision_col = getMaxCollisionCol(); // <列索引, 碰撞次数>

        // 打印调试信息
        std::cout << "R1 冲突点索引 : " << max_collision_row.first
                << ", 碰撞次数: " << max_collision_row.second << std::endl;
        std::cout << "R2 冲突点索引 : " << max_collision_col.first
                << ", 碰撞次数: " << max_collision_col.second << std::endl;

        /* 比较碰撞次数，选择碰撞次数更多的一方进行重规划 */
        // 如果行碰撞次数更多或相等，就将 R1 设置为冲突机器人
        if (max_collision_row.second >= max_collision_col.second && max_collision_row.second > 0)
        {
            blocking_conflict_r = CONFLICT_R1; // 设置冲突机器人为R1

            PathPlanner path_planner_r1(node_, r1_group_name);  // 左臂路径规划器实例化

            // 获取 R1 冲突点构型
            std::vector<double> obstacle_r1;
            for (int i = 0; i < 7; ++i) 
            {
                obstacle_r1.push_back(trajectory_r1.points[max_collision_row.first].position[i]);
            }
            // 获取 R1 初始位置 和 目标位置
            std::vector<double> start_r1, end_r1;
            for (int i = 0; i <7; ++i) 
            {
                start_r1.push_back(trajectory_r1.points[0].position[i]);
                end_r1.push_back(trajectory_r1.points[trajectory_r1.points.size() - 1].position[i]);
            }

            /* 设置 R1 冲突点构型为障碍物 */
            // 定义需要添加障碍物的连杆名称(只考虑可能发生碰撞的连杆)
            std::vector<std::string> r1_collision_links = {r1_links[r1_links.size()-2],
                                                            r1_links[r1_links.size()-1]};
            // 添加冲突点障碍物
            path_planner_r1.addConflictObstacle(obstacle_r1, r1_collision_links, 
                                                collision_threshold_, collision_threshold_, collision_threshold_);

            // R1 进行路径重规划，避开自身冲突点构型
            plan = path_planner_r1.planJointSpacePath(end_r1, start_r1);
            if (!plan.trajectory_.joint_trajectory.points.empty()) 
            {
                // 保存右臂重规划路径
                path_planner_r1.saveTrajectoryToFile(plan, package_path + "/data/trajectory_r1_replanning.txt");
                RCLCPP_INFO(node_->get_logger(), "R1 路径重规划成功，已保存到 trajectory_r1_replanning.txt");
            }

            // 清除 R1 冲突点障碍物
            path_planner_r1.clearConflictObstacles();
        }
        // 如果列碰撞次数更多，就将 R2 设置为冲突机器人
        else if (max_collision_col.second > max_collision_row.second && max_collision_col.second > 0)
        {
            blocking_conflict_r = CONFLICT_R2; // 设置冲突机器人为R2

            PathPlanner path_planner_r2(node_, r2_group_name); // 右臂路径规划器实例化

            // 获取 R2 冲突点构型
            std::vector<double> obstacle_r2;
            for (int i = 0; i < 7; ++i) 
            {
                obstacle_r2.push_back(trajectory_r2.points[max_collision_col.first].position[i]);
            }
            // 获取 R2 初始位置 和 目标位置
            std::vector<double> start_r2, end_r2;
            for (int i = 0; i < 7; ++i) 
            {
                start_r2.push_back(trajectory_r2.points[0].position[i]);
                end_r2.push_back(trajectory_r2.points[trajectory_r2.points.size() - 1].position[i]);
            }

            /* 设置 R2 冲突点构型为障碍物 */
            // 定义需要添加障碍物的连杆名称(只考虑可能发生碰撞的连杆)
            std::vector<std::string> r2_collision_links = {r2_links[r1_links.size()-2],
                                                            r2_links[r1_links.size()-1]};
            // 添加冲突点障碍物
            path_planner_r2.addConflictObstacle(obstacle_r2, r2_collision_links, 
                                                collision_threshold_, collision_threshold_, collision_threshold_);

            // R2 进行路径重规划，避开自身冲突点构型
            plan = path_planner_r2.planJointSpacePath(end_r2, start_r2);
            if (!plan.trajectory_.joint_trajectory.points.empty()) 
            {
                // 保存右臂重规划路径
                path_planner_r2.saveTrajectoryToFile(plan, package_path + "/data/trajectory_r2_replanning.txt");
                RCLCPP_INFO(node_->get_logger(), "R2 路径重规划成功，已保存到 trajectory_r2_replanning.txt");
            }

            // 清除 R2 冲突点障碍物
            path_planner_r2.clearConflictObstacles();
        }
    }
}

void DualArmCoordinator::saveCollisionMatrix(const std::string& file_path) 
{
    std::ofstream filecm(file_path);
    if (filecm.is_open()) 
    {
        for (int i = 0; i < collision_matrix_.rows(); ++i) 
        {
            for (int j = 0; j < collision_matrix_.cols(); ++j) 
            {
                filecm << collision_matrix_(i, j) << " ";
            }
            filecm << std::endl;
        }
        filecm.close();
    }
}

std::pair<int, int> DualArmCoordinator::getMaxCollisionRow()
{
    std::pair<int, int> max_collision_row; // 存储最大碰撞数量行的索引和数量
    int max_count = 0; // 存储碰撞数量的临时变量

    for (int i = 0; i < collision_matrix_.rows(); ++i) 
    {
        int collision_count = collision_matrix_.row(i).sum(); // 获取第 i+1 行的碰撞数量

        if (collision_count > max_count) 
        {
            max_count = collision_count;          // 更新最大碰撞数量
            max_collision_row.first = i;          // 更新索引
            max_collision_row.second = max_count; // 更新对应的碰撞数量
        }
    }
    return max_collision_row;
}

std::pair<int, int> DualArmCoordinator::getMaxCollisionCol()
{
    std::pair<int, int> max_collision_col; // 存储最大碰撞数量列的索引和数量
    int max_count = 0; // 存储碰撞数量的临时变量

    for (int i = 0; i < collision_matrix_.cols(); ++i) 
    {
        int collision_count = collision_matrix_.col(i).sum(); // 获取第 i+1 列的碰撞数量

        if (collision_count > max_count) 
        {
            max_count = collision_count;          // 更新最大碰撞数量
            max_collision_col.first = i;          // 更新索引
            max_collision_col.second = max_count; // 更新对应的碰撞数量
        }
    }
    return max_collision_col;
}

void DualArmCoordinator::coordinate(const Eigen::MatrixXd& cartesian_r1, const Eigen::MatrixXd& cartesian_r2,
                                    const PathTrackingPlanning::Trajectory& trajectory_r1,
                                    const PathTrackingPlanning::Trajectory& trajectory_r2) 
{
    // 0. 初始化所有变量
    initializeVariables();

    // 1. 生成碰撞矩阵
    generateCollisionMatrix(cartesian_r1, cartesian_r2);

    // 2. 检查并发运动轨迹是否发生碰撞
    if (isDiagCollision())
    {
        // 3. 生成碰撞边界矩阵
        generateCollisionBoundaries();

        // 4. 生成碰撞矩阵向量
        generateCollisionMatrixVector();
        
        // 5. 确定优先级和阻塞
        determinePriorityAndDeadlock();

        // 6. 判断是否发生阻塞
        if (deadlock_flag) // 如果阻塞，则进行路径重规划
        {
            RCLCPP_WARN(node_->get_logger(), "发生阻塞，进行路径重规划...");

            // 确定阻塞情况下的优先级
            determineBlockingConflictR();

            // 执行阻塞重规划
            blockingReplanning(trajectory_r1, trajectory_r2);
        }
        else // 如果无阻塞，则计算协调因子
        {
            calculateCoordFactors();
        }
    }
    else
    {
        RCLCPP_INFO(node_->get_logger(), "碰撞矩阵对角线无干涉，不进行协调.");
    }
}