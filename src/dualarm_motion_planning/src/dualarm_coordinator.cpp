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
    r1_arc_length_ = 0.0; // 左臂笛卡尔轨迹弧长
    r2_arc_length_ = 0.0; // 右臂笛卡尔轨迹弧长
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

    // 获取动态的连杆数量
    size_t n_links_r1 = r1_links.size();
    size_t n_links_r2 = r2_links.size();

    // 安全检查：确保有足够的连杆进行计算 (算法假设至少有两个连杆构成线段)
    if (n_links_r1 < 2 || n_links_r2 < 2) {
        RCLCPP_ERROR(node_->get_logger(), "Not enough links for collision detection!");
        return;
    }

    std::array<double, 3> s1_v1, s1_v2, s2_v1, s2_v2; // 定义碰撞检测线段
    // 遍历每一对样本，检测碰撞
    for (int i = 0; i < sample_r1; ++i) 
    {
        for (int j = 0; j < sample_r2; ++j) 
        {
            bool flag_collision = false;
            // 遍历部分可能存在碰撞的关节对（提高构建碰撞矩阵的效率）
            // 这里假设只检测末端的3段连杆，如果连杆数少于4，需要调整循环次数
            size_t check_depth = 3;
            if (n_links_r1 < 4 || n_links_r2 < 4) check_depth = std::min(n_links_r1, n_links_r2) - 1;

            for (size_t a = 0; a < check_depth; ++a) 
            {
                for (size_t b = 0; b < check_depth; ++b) 
                {
                    // 动态索引：(n_links_r1 - 1) 是最后一个连杆的索引
                    // s1_v1 对应 Link[N-1-a]，s1_v2 对应 Link[N-2-a]，构成线段
                    
                    size_t idx_r1_curr = n_links_r1 - 1 - a;
                    size_t idx_r1_prev = n_links_r1 - 2 - a;
                    
                    size_t idx_r2_curr = n_links_r2 - 1 - b;
                    size_t idx_r2_prev = n_links_r2 - 2 - b;

                    // 左臂关节a的线段端点（link prev 到 link curr）
                    s1_v1 = {
                        cartesian_r1(3*idx_r1_curr+0, i),
                        cartesian_r1(3*idx_r1_curr+1, i),
                        cartesian_r1(3*idx_r1_curr+2, i) 
                    };
                    s1_v2 = {
                        cartesian_r1(3*idx_r1_prev+0, i),
                        cartesian_r1(3*idx_r1_prev+1, i),
                        cartesian_r1(3*idx_r1_prev+2, i)
                    };
                    // 右臂关节b的线段端点
                    s2_v1 = {
                        cartesian_r2(3*idx_r2_curr+0, j),
                        cartesian_r2(3*idx_r2_curr+1, j),
                        cartesian_r2(3*idx_r2_curr+2, j)
                    };
                    s2_v2 = {
                        cartesian_r2(3*idx_r2_prev+0, j),
                        cartesian_r2(3*idx_r2_prev+1, j),
                        cartesian_r2(3*idx_r2_prev+2, j)
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

// 并发运动轨迹碰撞检测
bool DualArmCoordinator::isDiagCollision() 
{
    // 获取行数 (N_s1, R1 的采样点数) 和 列数 (N_s2, R2 的采样点数)
    int n_s1 = collision_matrix_.rows();
    int n_s2 = collision_matrix_.cols();

    // 1. 检查共同运动阶段（对角线部分）
    int min_steps = std::min(n_s1, n_s2);
    for (int k = 0; k < min_steps; ++k) 
    {
        if (collision_matrix_(k, k) == 1) 
        {
            diag_collision_flag = true; // 发现碰撞
            return true;
        }
    }

    // 2. 检查单臂保持阶段（延伸部分）
    // 判断哪种情形适用
    if (n_s1 <= n_s2) 
    {
        // === 情形 1 (Case 1): N_s1 <= N_s2 ===
        // R1 先完成运动 (或同时完成)，保持在最后一个姿态 (n_s1 - 1)，R2 继续运动
        int fixed_row = n_s1 - 1;
        
        for (int k = n_s1; k < n_s2; ++k) 
        {
            if (collision_matrix_(fixed_row, k) == 1) 
            {
                diag_collision_flag = true;
                return true;
            }
        }
    } 
    else 
    {
        // === 情形 2 (Case 2): N_s1 > N_s2 ===
        // R2 先完成运动，保持在最后一个姿态 (n_s2 - 1)，R1 继续运动
        int fixed_col = n_s2 - 1;
        
        for (int k = n_s2; k < n_s1; ++k) 
        {
            if (collision_matrix_(k, fixed_col) == 1) 
            {
                diag_collision_flag = true;
                return true;
            }
        }
    }
    
    // 如果遍历完所有轨迹点都没有碰撞
    diag_collision_flag = false;
    return diag_collision_flag; 
}

void DualArmCoordinator::calculateCoordFactors()
{
    int r1_sample_size = collision_matrix_.rows();
    int r2_sample_size = collision_matrix_.cols();
    double collision_avoidance_margin = sample_size_max_ * sample_time_; // 碰撞避免的安全边界

    // R1优先，协调R2（寻找最严格的下边界约束）
    if (prior_r1_)
    {
        double max_exceed = 0.0;
        for (int i = 0; i < r1_sample_size; ++i) // 遍历所有R1样本，寻找最大超出下边界的值
        {
            if (collision_boundary_r1_(0, i) == 1) // 仅考虑发生碰撞的样本
            {
                int j_lower = collision_boundary_r1_(1, i);
                double exceed = i - j_lower;
                if (exceed > max_exceed) 
                {
                    max_exceed = exceed;
                }
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
        for (int j = 0; j < r2_sample_size; ++j) 
        {
            if (collision_boundary_r2_(0, j) == 1) 
            {
                int i_lower = collision_boundary_r2_(1, j);
                double exceed = j - i_lower;
                if (exceed > max_exceed) 
                {
                    max_exceed = exceed;
                }
            }
        }
        delay_time_r1_ = (max_exceed + collision_avoidance_margin) * sample_time_;
    }
}

// 计算笛卡尔轨迹弧长 (基于末端 Link)
double DualArmCoordinator::calculateCartesianArcLength(const Eigen::MatrixXd& cartesian_data)
{
    double length = 0.0;
    int cols = cartesian_data.cols(); // 样本数
    int rows = cartesian_data.rows(); // 笛卡尔数据总行数 (3 * link_num)
    
    if (cols < 2 || rows < 3) return 0.0;

    // 动态索引：假设矩阵最后3行存储的是末端执行器的坐标
    int idx_x = rows - 3;
    int idx_y = rows - 2;
    int idx_z = rows - 1;

    for (int i = 0; i < cols - 1; ++i)
    {
        double dx = cartesian_data(idx_x, i+1) - cartesian_data(idx_x, i);
        double dy = cartesian_data(idx_y, i+1) - cartesian_data(idx_y, i);
        double dz = cartesian_data(idx_z, i+1) - cartesian_data(idx_z, i);
        length += std::sqrt(dx*dx + dy*dy + dz*dz);
    }
    return length;
}

// 引入空间跨度（弧长）的仲裁机制
void DualArmCoordinator::determineBlockingConflictR()
{
    // 打印调试信息
    std::cout << "[Arbitration] Arc Lengths -> R1: " << r1_arc_length_ 
              << " m, R2: " << r2_arc_length_ << " m." << std::endl;

    // 1. 明显的拓扑单向阻挡
    if ((!collision_matrix_vector_[0] && !collision_matrix_vector_[1] && collision_matrix_vector_[2] && collision_matrix_vector_[3])      
        ||(!collision_matrix_vector_[0] && collision_matrix_vector_[1] && collision_matrix_vector_[2] && collision_matrix_vector_[3])     
        ||(collision_matrix_vector_[0] && !collision_matrix_vector_[1] && collision_matrix_vector_[2] && collision_matrix_vector_[3]))    
    {
        blocking_conflict_r = CONFLICT_R1; 
    }
    else if ((collision_matrix_vector_[0] && collision_matrix_vector_[1] && !collision_matrix_vector_[2] && !collision_matrix_vector_[3])      
            ||(collision_matrix_vector_[0] && collision_matrix_vector_[1] && !collision_matrix_vector_[2] && collision_matrix_vector_[3])      
            ||(collision_matrix_vector_[0] && collision_matrix_vector_[1] && collision_matrix_vector_[2] && !collision_matrix_vector_[3]))     
    {
        blocking_conflict_r = CONFLICT_R2; 
    }
    else // 双方均有阻挡可能
    {
        // 2. 拓扑模糊或全阻塞 (CONFLICT_R1_R2 候选)
        // 引入 "空间迂回势能" (Spatial Detour Potential) 仲裁
        
        // 如果 R1 几乎不动 (静态障碍)，强制选 R2 重规划
        if (r1_arc_length_ < ARC_LENGTH_THRESHOLD && r2_arc_length_ >= ARC_LENGTH_THRESHOLD)
        {
            RCLCPP_INFO(node_->get_logger(), "Arbitration: R1 is static/short (%f m). Choosing R2.", r1_arc_length_);
            blocking_conflict_r = CONFLICT_R2;
        }
        // 如果 R2 几乎不动，强制选 R1 重规划
        else if (r2_arc_length_ < ARC_LENGTH_THRESHOLD && r1_arc_length_ >= ARC_LENGTH_THRESHOLD)
        {
            RCLCPP_INFO(node_->get_logger(), "Arbitration: R2 is static/short (%f m). Choosing R1.", r2_arc_length_);
            blocking_conflict_r = CONFLICT_R1;
        }
        else
        {
            // 3. 双方都动或都不动，回退到原来的逻辑 (CONFLICT_R1_R2)，后续比较碰撞次数
            blocking_conflict_r = CONFLICT_R1_R2;
        }
    }
}

// 返回 bool，指示是否重规划成功
bool DualArmCoordinator::blockingReplanning(const PathTrackingPlanning::Trajectory& trajectory_r1,
                                            const PathTrackingPlanning::Trajectory& trajectory_r2)
{
    // 逻辑 A: R1 重规划
    if (blocking_conflict_r == CONFLICT_R1)
    {
        PathPlanner path_planner_r1(node_, r1_group_name);
        
        // 使用拓扑质心法获取 R1 冲突点
        std::pair<int, int> max_collision_row = getMaxCollisionRow(); 
        
        std::cout << "[Replanning] R1 Selected. Conflict Index: " << max_collision_row.first 
                  << ", Count: " << max_collision_row.second << std::endl;

        // 构建障碍物
        std::vector<double> obstacle_r1;
        for (int i = 0; i < 7; ++i) obstacle_r1.push_back(trajectory_r1.points[max_collision_row.first].position[i]);
        
        std::vector<double> start_r1, end_r1;
        for (int i = 0; i <7; ++i) {
            start_r1.push_back(trajectory_r1.points[0].position[i]);
            end_r1.push_back(trajectory_r1.points[trajectory_r1.points.size() - 1].position[i]);
        }

        std::vector<std::string> r1_collision_links = {r1_links[r1_links.size()-1]};
        path_planner_r1.addConflictObstacle(obstacle_r1, r1_collision_links, 
                                            0.5*collision_threshold_, 0.5*collision_threshold_, 0.5*collision_threshold_);

        // 规划
        plan = path_planner_r1.planJointSpacePath(end_r1, start_r1);
        path_planner_r1.clearConflictObstacles(); // 无论成功失败都清理

        if (!plan.trajectory_.joint_trajectory.points.empty()) 
        {
            path_planner_r1.saveTrajectoryToFile(plan, package_path + "/data/trajectory_r1_replanning.txt");
            RCLCPP_INFO(node_->get_logger(), "R1 Replanning Success.");
            return true;
        }
        else 
        {
            RCLCPP_WARN(node_->get_logger(), "R1 Replanning Failed.");
            return false;
        }
    }
    // 逻辑 B: R2 重规划
    else if (blocking_conflict_r == CONFLICT_R2)
    {
        PathPlanner path_planner_r2(node_, r2_group_name);
        std::pair<int, int> max_collision_col = getMaxCollisionCol(); 
        
        std::cout << "[Replanning] R2 Selected. Conflict Index: " << max_collision_col.first 
                  << ", Count: " << max_collision_col.second << std::endl;

        std::vector<double> obstacle_r2;
        for (int i = 0; i < 7; ++i) obstacle_r2.push_back(trajectory_r2.points[max_collision_col.first].position[i]);
        
        std::vector<double> start_r2, end_r2;
        for (int i = 0; i < 7; ++i) {
            start_r2.push_back(trajectory_r2.points[0].position[i]);
            end_r2.push_back(trajectory_r2.points[trajectory_r2.points.size() - 1].position[i]);
        }

        std::vector<std::string> r2_collision_links = {r2_links[r2_links.size()-1]};
        path_planner_r2.addConflictObstacle(obstacle_r2, r2_collision_links, 
                                            0.5*collision_threshold_, 0.5*collision_threshold_, 0.5*collision_threshold_);

        plan = path_planner_r2.planJointSpacePath(end_r2, start_r2);
        path_planner_r2.clearConflictObstacles();

        if (!plan.trajectory_.joint_trajectory.points.empty()) 
        {
            path_planner_r2.saveTrajectoryToFile(plan, package_path + "/data/trajectory_r2_replanning.txt");
            RCLCPP_INFO(node_->get_logger(), "R2 Replanning Success.");
            return true;
        }
        else
        {
            RCLCPP_WARN(node_->get_logger(), "R2 Replanning Failed.");
            return false;
        }
    }
    // 逻辑 C: 比较碰撞次数 (Level 3 Arbitration)
    else if (blocking_conflict_r == CONFLICT_R1_R2)
    {
        std::pair<int, int> max_collision_row = getMaxCollisionRow(); 
        std::pair<int, int> max_collision_col = getMaxCollisionCol(); 

        // 如果 R1 碰撞更多或相等，首选 R1
        if (max_collision_row.second >= max_collision_col.second && max_collision_row.second > 0)
        {
            blocking_conflict_r = CONFLICT_R1; // 临时指定为 R1，递归调用
            return blockingReplanning(trajectory_r1, trajectory_r2);
        }
        else if (max_collision_col.second > max_collision_row.second && max_collision_col.second > 0)
        {
            blocking_conflict_r = CONFLICT_R2; // 临时指定为 R2，递归调用
            return blockingReplanning(trajectory_r1, trajectory_r2);
        }
    }
    
    return false; // 如果前面都没进入（理论上不应发生）
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

/*
    获取冲突区间的几何中心行（优化版：基于冲突时空质心）
    2025.12.9 理论优化：
    不再寻找碰撞次数最多的行，而是寻找冲突时间段的中点。
    这能最大程度确保生成的障碍物距离起止点都有足够的物理安全距离，
    避免梯形速度曲线导致的起止点采样密集引发的自碰撞问题。
*/
std::pair<int, int> DualArmCoordinator::getMaxCollisionRow()
{
    std::pair<int, int> critical_conflict_row; 
    critical_conflict_row.first = -1;
    critical_conflict_row.second = 0;

    int n_samples = collision_matrix_.rows();
    int first_collision_idx = -1;
    int last_collision_idx = -1;

    // 1. 扫描碰撞区间：找到发生碰撞的最早和最晚时刻（索引）
    // 我们可以略过首尾极小部分(e.g., 1-2点)防止边缘噪声，但主要靠中点逻辑来保证安全
    for (int i = 1; i < n_samples - 1; ++i) 
    {
        if (collision_matrix_.row(i).sum() > 0) 
        {
            if (first_collision_idx == -1) first_collision_idx = i; // 记录起点
            last_collision_idx = i; // 不断更新终点
        }
    }

    // 2. 如果没有检测到任何碰撞（理论上进不来这里，因为只有isDiagCollision为真才调用）
    if (first_collision_idx == -1) 
    {
        // 兜底：如果没有找到区间，为了防止崩溃，返回一个安全的中值（或者直接返回-1）
        // 这里选择返回中间索引，作为一种尝试
        critical_conflict_row.first = n_samples / 2;
        return critical_conflict_row;
    }

    // 3. 计算冲突区间的几何中心（质心）
    int center_idx = first_collision_idx + (last_collision_idx - first_collision_idx) / 2;

    // 4. 验证中心点有效性并微调
    // 虽然理论上区间是连续的，但为了防止矩阵中有空洞，确认该行确实有碰撞
    // 如果中点没碰撞，就向两侧搜索最近的碰撞点
    if (collision_matrix_.row(center_idx).sum() == 0)
    {
        int offset = 1;
        while (center_idx - offset >= first_collision_idx || center_idx + offset <= last_collision_idx)
        {
            if (center_idx + offset <= last_collision_idx && collision_matrix_.row(center_idx + offset).sum() > 0)
            {
                center_idx = center_idx + offset;
                break;
            }
            if (center_idx - offset >= first_collision_idx && collision_matrix_.row(center_idx - offset).sum() > 0)
            {
                center_idx = center_idx - offset;
                break;
            }
            offset++;
        }
    }

    // 5. 赋值结果
    critical_conflict_row.first = center_idx;
    critical_conflict_row.second = collision_matrix_.row(center_idx).sum(); // 记录碰撞数用于调试

    // 打印调试信息，验证是否远离了 0
    // std::cout << "[DEBUG] R1 Conflict Range: [" << first_collision_idx << ", " << last_collision_idx 
    //           << "], Center: " << center_idx << std::endl;

    return critical_conflict_row;
}

/*
    获取冲突区间的几何中心列
*/
std::pair<int, int> DualArmCoordinator::getMaxCollisionCol()
{
    std::pair<int, int> critical_conflict_col; 
    critical_conflict_col.first = -1;
    critical_conflict_col.second = 0;

    int n_samples = collision_matrix_.cols();
    int first_collision_idx = -1;
    int last_collision_idx = -1;

    // 1. 扫描碰撞区间
    for (int i = 1; i < n_samples - 1; ++i) 
    {
        if (collision_matrix_.col(i).sum() > 0) 
        {
            if (first_collision_idx == -1) first_collision_idx = i;
            last_collision_idx = i;
        }
    }

    // 2. 兜底
    if (first_collision_idx == -1) 
    {
        critical_conflict_col.first = n_samples / 2;
        return critical_conflict_col;
    }

    // 3. 计算中心
    int center_idx = first_collision_idx + (last_collision_idx - first_collision_idx) / 2;

    // 4. 验证并微调
    if (collision_matrix_.col(center_idx).sum() == 0)
    {
        int offset = 1;
        while (center_idx - offset >= first_collision_idx || center_idx + offset <= last_collision_idx)
        {
            if (center_idx + offset <= last_collision_idx && collision_matrix_.col(center_idx + offset).sum() > 0)
            {
                center_idx = center_idx + offset;
                break;
            }
            if (center_idx - offset >= first_collision_idx && collision_matrix_.col(center_idx - offset).sum() > 0)
            {
                center_idx = center_idx - offset;
                break;
            }
            offset++;
        }
    }

    // 5. 赋值
    critical_conflict_col.first = center_idx;
    critical_conflict_col.second = collision_matrix_.col(center_idx).sum();

    // std::cout << "[DEBUG] R2 Conflict Range: [" << first_collision_idx << ", " << last_collision_idx 
    //           << "], Center: " << center_idx << std::endl;

    return critical_conflict_col;
}

// 核心协调流程，加入反馈重试逻辑
void DualArmCoordinator::coordinate(const Eigen::MatrixXd& cartesian_r1, const Eigen::MatrixXd& cartesian_r2,
                                    const PathTrackingPlanning::Trajectory& trajectory_r1,
                                    const PathTrackingPlanning::Trajectory& trajectory_r2) 
{
    initializeVariables();
    generateCollisionMatrix(cartesian_r1, cartesian_r2);

    if (isDiagCollision())
    {
        generateCollisionBoundaries();
        generateCollisionMatrixVector();
        determinePriorityAndDeadlock();

        if (deadlock_flag) 
        {
            RCLCPP_WARN(node_->get_logger(), "Blocking detected. Initiating replanning...");

            // 计算空间跨度 (弧长)
            r1_arc_length_ = calculateCartesianArcLength(cartesian_r1);
            r2_arc_length_ = calculateCartesianArcLength(cartesian_r2);

            // 1. 初次仲裁 (基于拓扑 + 弧长势能)
            determineBlockingConflictR();

            // 2. 执行重规划 (带反馈)
            bool success = blockingReplanning(trajectory_r1, trajectory_r2);

            // 3. 失败反转机制 (角色互换)
            if (!success)
            {
                RCLCPP_ERROR(node_->get_logger(), "Primary replanning failed. Attempting ROLE REVERSAL...");
                
                // 切换冲突对象
                if (blocking_conflict_r == CONFLICT_R1) blocking_conflict_r = CONFLICT_R2;
                else if (blocking_conflict_r == CONFLICT_R2) blocking_conflict_r = CONFLICT_R1;

                // 再次尝试重规划
                bool retry_success = blockingReplanning(trajectory_r1, trajectory_r2);
                
                if (retry_success) {
                    RCLCPP_INFO(node_->get_logger(), "Secondary replanning (Role Reversal) SUCCESS.");
                } else {
                    RCLCPP_ERROR(node_->get_logger(), "Both replanning attempts FAILED. Critical Deadlock.");
                }
            }
        }
        else // 无阻塞，计算协调因子
        {
            calculateCoordFactors();
        }
    }
}