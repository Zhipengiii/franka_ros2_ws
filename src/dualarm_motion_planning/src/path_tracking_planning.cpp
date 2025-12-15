#include "path_tracking_planning.h"

// 构造函数
PathTrackingPlanning::PathTrackingPlanning(double v_actual_max, double a_actual_max, Vector7d v_0_in, Eigen::MatrixXd q_path_in)
{
    initializeTrajectory(); 

    for (size_t i = 0; i < 7; ++i) {
        v_max_[i] = v_actual_max;
        a_max_[i] = a_actual_max;
        v_0_[i] = v_0_in[i]; 
    }

    initializeMotion(q_path_in);
}

void PathTrackingPlanning::initializeTrajectory()
{
    trajectory.points.clear();
}

// ============================================================
// 核心函数：初始化运动、构建样条、时间缩放
// ============================================================
void PathTrackingPlanning::initializeMotion(const Eigen::MatrixXd& q_path_in)
{
    waypoints_.clear();
    int n_raw = q_path_in.rows();
    if (n_raw == 0) return;

    // ------------------------------------------------------------
    // 第一步：路径点预处理（Pre-processing）
    // 目的：去除重复点或距离过近的点，防止样条计算时分母为零导致数值爆炸
    // ------------------------------------------------------------
    // 添加 .transpose()，将行向量(1x7)转为列向量(7x1)
    waypoints_.push_back(q_path_in.row(0).transpose());

    for (int i = 1; i < n_raw; ++i) {
        // 先计算距离，注意 transpose
        double dist = (q_path_in.row(i).transpose() - waypoints_.back()).norm();
        
        if (dist > 1e-4) {
            // 这里必须加 .transpose()，否则会触发 resize 错误！
            waypoints_.push_back(q_path_in.row(i).transpose());
        }
    }
    
    // 检查终点
    if ((q_path_in.bottomRows(1).transpose() - waypoints_.back()).norm() > 1e-4) {
        // 这里的 bottomRows(1) 也是 1x7，必须转置
        waypoints_.push_back(q_path_in.bottomRows(1).transpose());
    } else {
        // 强制覆盖最后一点，确保精确，同样需要转置
        waypoints_.back() = q_path_in.bottomRows(1).transpose();
    }

    int n_via = waypoints_.size();
    if (n_via < 2) return; 

    // ------------------------------------------------------------
    // 第二步：激进的时间初始化 (Aggressive Time Initialization)
    // 目的：为了让规划出的轨迹尽可能快，我们初始假设机械臂能以 speed_factor 的 v_max 运行。
    // 原理：算法后续会进行“只增不减”的时间缩放。如果初始给的时间很短（速度很快），
    //       缩放算法会自动把它拉长到刚好满足约束的极限位置。
    // ------------------------------------------------------------
    time_knots_.assign(n_via, 0.0);
    double global_min_vmax = v_max_.minCoeff(); 
    double speed_factor = 1.5; // 通过调整该因子可以控制轨迹的激进程度
    double aggressive_vel = global_min_vmax * speed_factor; 

    for (int i = 1; i < n_via; ++i) {
        double dist = (waypoints_[i] - waypoints_[i-1]).norm();
        double dt = std::max(dist / aggressive_vel, 0.01); 
        time_knots_[i] = time_knots_[i-1] + dt;
    }

    // ------------------------------------------------------------
    // 第三步：迭代时间缩放 (Iterative Time Scaling)
    // 目的：检查当前时间下的样条是否超速/超加速度。如果超限，拉长总时间。
    // ------------------------------------------------------------
    double scaling_factor = 1.0;
    const int max_iterations = 50; 
    bool limits_satisfied = false;
    
    joint_splines_.resize(7); 

    for (int iter = 0; iter < max_iterations; ++iter) {
        // 应用当前缩放因子，得到新的时间节点
        std::vector<double> scaled_times = time_knots_;
        for (double& t : scaled_times) t *= scaling_factor;

        // 计算每个关节的样条系数
        for (int j = 0; j < 7; ++j) {
            std::vector<double> positions;
            for (const auto& wp : waypoints_) positions.push_back(wp[j]);
            joint_splines_[j] = computeSpline(scaled_times, positions);
        }

        // 检查速度和加速度约束
        double max_v_ratio = 0.0; 
        double max_a_ratio = 0.0; 
        
        // 采用细粒度采样检查 (每段样条采样5个点)
        double check_dt = (scaled_times.back() - scaled_times.front()) / (n_via * 5.0);
        
        for (double t = 0; t <= scaled_times.back(); t += check_dt) {
            for (int j = 0; j < 7; ++j) {
                // 找到当前时间 t 所在的样条段
                int idx = 0;
                int n_seg = joint_splines_[j].size();
                if (t >= joint_splines_[j].back().x) idx = n_seg - 1;
                else {
                    for(int k=0; k<n_seg-1; ++k) {
                        if(t >= joint_splines_[j][k].x && t < joint_splines_[j][k+1].x) {
                            idx = k; break;
                        }
                    }
                }

                // 根据样条公式计算速度 v 和加速度 a
                const auto& s = joint_splines_[j][idx];
                double dt_samp = t - s.x;
                double v = s.b + 2.0 * s.c * dt_samp + 3.0 * s.d * dt_samp * dt_samp;
                double a = 2.0 * s.c + 6.0 * s.d * dt_samp;

                // 更新最大超限比例
                max_v_ratio = std::max(max_v_ratio, std::abs(v) / v_max_[j]);
                max_a_ratio = std::max(max_a_ratio, std::abs(a) / a_max_[j]);
            }
        }

        // 检查是否所有关节均满足约束
        if (max_v_ratio <= 1.0 && max_a_ratio <= 1.0) {
            time_knots_ = scaled_times; 
            limits_satisfied = true;
            break;
        }

        // 计算更新后的缩放因子
        double required_scale = std::max(max_v_ratio, std::sqrt(max_a_ratio));
        scaling_factor *= (required_scale * 1.01); 
    }

    if (!limits_satisfied) {
        std::cerr << "[WARNING] Constraints not strictly met after max iterations. Using safest guess." << std::endl;
        for (double& t : time_knots_) t *= scaling_factor;
    }
}

PathTrackingPlanning::Trajectory PathTrackingPlanning::startPlanning(double sample_time)
{
    trajectory.points.clear();

    if (time_knots_.empty() || joint_splines_.empty()) {
        return trajectory;
    }

    double total_time = time_knots_.back();
    double current_time = 0.0;

    while (current_time <= total_time)
    {
        TrajectoryPoint point;
        
        for (int j = 0; j < 7; ++j) {
            double p, v;
            sampleSpline(current_time, joint_splines_[j], p, v);
            point.position[j] = p;
            point.velocity[j] = v;
        }

        trajectory.points.push_back(point);
        current_time += sample_time;
    }

    // 修正起点
    if (!trajectory.points.empty()) {
        for(int j=0; j<7; ++j) {
            trajectory.points[0].position[j] = waypoints_[0][j];
        }
    }

    // 修正终点
    TrajectoryPoint last_point;
    for (int j = 0; j < 7; ++j) {
        last_point.position[j] = waypoints_.back()[j];
        last_point.velocity[j] = 0.0; 
    }
    
    if (trajectory.points.empty()) {
        trajectory.points.push_back(last_point);
    } else {
        double end_time_diff = total_time - (trajectory.points.size()-1)*sample_time;
        if (end_time_diff > 1e-6) {
             trajectory.points.push_back(last_point); 
        } else {
             trajectory.points.back() = last_point;   
        }
    }

    return trajectory;
}

size_t PathTrackingPlanning::getTrajectorySize()
{
    return trajectory.points.size();
}

// ============================================================
// 数学核心：三次样条插值 (Natural Cubic Spline)
// 求解方程组 Ax = B，得到每段多项式的系数
// ============================================================
std::vector<PathTrackingPlanning::SplineSegment> PathTrackingPlanning::computeSpline(
    const std::vector<double>& times, const std::vector<double>& positions)
{
    int n = times.size() - 1; // 样条段数量
    if (n < 1) return {};

    std::vector<double> a = positions; 
    std::vector<double> h(n), alpha(n);
    
    // 计算时间步长 h[i] = t[i+1] - t[i]
    for (int i = 0; i < n; ++i) h[i] = times[i + 1] - times[i];

    // 计算线性方程组的右边部分 (Alpha)
    // 这一步基于平滑条件：S_i''(t_i) = S_{i-1}''(t_i) -> 加速度连续
    for (int i = 1; i < n; ++i)
        alpha[i] = 3.0 / h[i] * (a[i + 1] - a[i]) - 3.0 / h[i - 1] * (a[i] - a[i - 1]);

    // 解三对角线性方程组，得到 c[i] 系数
    std::vector<double> c(n + 1), l(n + 1), mu(n + 1), z(n + 1);
    
    // 边界条件：自然样条 (Natural Spline)
    // 假设起点和终点的二阶导数(加速度)为 0
    l[0] = 1.0; mu[0] = 0.0; z[0] = 0.0;

    // 前向消元
    for (int i = 1; i < n; ++i) {
        l[i] = 2.0 * (times[i + 1] - times[i - 1]) - h[i - 1] * mu[i - 1];
        mu[i] = h[i] / l[i];
        z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / l[i];
    }

    // 终点边界条件 ：终点二阶导数为 0
    l[n] = 1.0; z[n] = 0.0; c[n] = 0.0;

    // 回代求解 (Back Substitution)
    // 得到所有段的 c, b, d 系数
    std::vector<double> b(n), d(n);
    std::vector<SplineSegment> splines(n);

    for (int j = n - 1; j >= 0; --j) {
        c[j] = z[j] - mu[j] * c[j + 1];
        b[j] = (a[j + 1] - a[j]) / h[j] - h[j] * (c[j + 1] + 2.0 * c[j]) / 3.0;
        d[j] = (c[j + 1] - c[j]) / (3.0 * h[j]);
        
        splines[j].a = a[j];
        splines[j].b = b[j];
        splines[j].c = c[j];
        splines[j].d = d[j];
        splines[j].x = times[j];
    }

    return splines;
}

// 样条曲线采样函数
void PathTrackingPlanning::sampleSpline(double t, const std::vector<SplineSegment>& spline, double& pos, double& vel)
{
    int n = spline.size();
    int idx = 0;

    // 二分查找或线性查找确定当前时间 t 处于哪一段 spline
    if (t <= spline[0].x) idx = 0;
    else if (t >= spline.back().x) {
        idx = n - 1;
    } 
    else {
        for (int i = 0; i < n; ++i) {
            double next_t = (i == n - 1) ? 1e9 : spline[i+1].x;
            if (t >= spline[i].x && t < next_t) {
                idx = i;
                break;
            }
        }
    }

    const auto& s = spline[idx];
    double dt = t - s.x;

    // 处理时间超出范围的情况
    if (idx < n - 1) {
    } else {
       if(dt > (time_knots_.back() - time_knots_[n-1])) 
           dt = time_knots_.back() - time_knots_[n-1];
    }

    // 代入三次多项式公式计算位置和速度
    // Position = a + b*t + c*t^2 + d*t^3
    // Velocity = b + 2*c*t + 3*d*t^2 (求导)
    pos = s.a + s.b * dt + s.c * dt * dt + s.d * dt * dt * dt;
    vel = s.b + 2.0 * s.c * dt + 3.0 * s.d * dt * dt;
}

void PathTrackingPlanning::plotTrajectory(const Trajectory& trajectory, 
                                        const std::string& data_dir, 
                                        const std::string& file_name)
{
    std::string command = "mkdir -p " + data_dir;
    int ret = std::system(command.c_str());
    (void)ret; 

    std::string temp_filename = data_dir + "/" + file_name + ".dat";
    std::string plot_filename = data_dir + "/" + file_name + ".png";

    std::ofstream temp_file(temp_filename);
    if (temp_file.is_open())
    {
        for (size_t k=0; k < trajectory.points.size(); ++k)
        {
            const auto& point = trajectory.points[k];
            temp_file << k << " "; 
            for (size_t i = 0; i < 7; ++i) temp_file << point.position[i] << " ";
            for (size_t i = 0; i < 7; ++i) temp_file << point.velocity[i] << " ";
            temp_file << "\n";
        }
        temp_file.close();

        std::string plot_command = "gnuplot -e \"set terminal png size 1000,800; set output '" + plot_filename + "'; "
                                    "set multiplot layout 2,1 title 'Trajectory Analysis'; "
                                    "set grid; "
                                    "set ylabel 'Position (rad)'; "
                                    "plot for [i=2:8] '" + temp_filename + "' using 0:i with lines title 'J'.(i-1); "
                                    "set ylabel 'Velocity (rad/s)'; "
                                    "plot for [i=9:15] '" + temp_filename + "' using 0:i with lines title 'V'.(i-8); "
                                    "unset multiplot\"";
        int plot_ret = std::system(plot_command.c_str());
        (void)plot_ret;
    }
}

void PathTrackingPlanning::saveTrajectoryData(std::string position_file_path, std::string velocity_file_path)
{
    std::ofstream position_file(position_file_path);
    std::ofstream velocity_file(velocity_file_path);

    if (!position_file.is_open() || !velocity_file.is_open()) return;

    for (const auto& point : trajectory.points)
    {
        for (size_t i = 0; i < 7; ++i) position_file << point.position[i] << " ";
        position_file << std::endl;

        for (size_t i = 0; i < 7; ++i) velocity_file << point.velocity[i] << " ";
        velocity_file << std::endl;
    }

    position_file.close();
    velocity_file.close();
    std::cout << "[INFO] Trajectory data saved to " << position_file_path << std::endl;
}