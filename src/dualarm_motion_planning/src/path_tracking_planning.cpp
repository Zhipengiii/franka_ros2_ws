#include "path_tracking_planning.h"

const double kDeltaQMotionFinished = 1e-6;

// 运动生成类构造函数
PathTrackingPlanning::PathTrackingPlanning(double v_actual_max, double a_actual_max, Vector7d v_0_, Eigen::MatrixXd q_path_)
{
    initializeTrajectory(); // 初始化轨迹

    // 存储初始位置构型
    for (size_t i = 0; i < 7; ++i)
    {
        init_position[i] = q_path_(0, i);
    }

    for (size_t i = 0; i < 7; ++i)
    {
        v_max[i] = v_actual_max;
        a_max[i] = a_actual_max;
    }

    initializeMotion(v_0_,q_path_); // 初始化运动
}

// 初始化运动函数，确定路径中的方向变化点，以及梯形速度曲线参数初始化
void PathTrackingPlanning::initializeMotion(Vector7d v_0_, Eigen::MatrixXd q_path_)
{
    initializeTrajectory(); // 初始化轨迹

    q_path = q_path_;
    n_via = q_path.rows(); // 路径点总数

    calculateChangeDirection(); // 确定方向转换点

    i_via = 0; // 当前路径点索引
    motion_finished = false; // 运动规划结束标志
    for (size_t i = 0; i < n_dof; i++) // 关节遍历
    {
        q_f[i] = q_path(i_via_change_direction(0,i),i); // 将第一个转向路径点的关节配置作为目标构型配置
    }
    q_0 = q_path.row(i_via);        // 当前的路径点
    v_0 = v_0_;                     // 初始速度
    q_via = q_path.row(i_via+1);    // 下一个路径点

    // 路径点参数更新
    updateParameters(); 

    q_current = q_0;
    v_current = v_0;

    t_f_sync = 0; // 总的同步运动时间

    // 梯形速度曲线参数初始化
    acc.setZero();
    vm.setZero();
    t1.setZero();
    t2.setZero();
    tf.setZero();
}

// 计算方向转换点函数
void PathTrackingPlanning::calculateChangeDirection()
{
    n_change_direction = Eigen::VectorXi::Zero(7);   // 每个关节改变方向的次数
    i_via_change_direction = Eigen::MatrixXi::Ones(n_via, 7) * (n_via-1);
    i_change_direction = Eigen::VectorXi::Zero(7);  // 方向切换点索引

    /*
        i_via_change_direction : 存储每个关节转向时对应的路径点索引
        for example:
        n_via = 5
        (4 2 4 4 4 4 4
         4 3 4 4 4 4 4
         4 4 4 4 4 4 4
         4 4 4 4 4 4 4
         4 4 4 4 4 4 4）
        只有第二关节出现了关节转向的情况，且发生了两次转向，分别是第3个 和 第4个路径点
    */

    /* 记录每个关节的方向变化情况 */
    for (size_t j = 0; j < 7; j++) // 关节遍历
    {
        int prev_sign = 0; // 前一个非零差的方向，0表示未初始化

        for (size_t i = 1; i < n_via; i++) // 路径点遍历
        {
            double current_diff = q_path(i, j) - q_path(i-1, j);
            if (current_diff != 0)
            {
                int current_sign = (current_diff > 0) ? 1 : -1; // 1 表示正向，-1表示反向
                if (prev_sign != 0 && (current_sign != prev_sign))
                {
                    // 方向变换点发生在 i-1 的位置
                    i_via_change_direction(n_change_direction[j],j) = i-1;
                    n_change_direction[j] = n_change_direction[j] + 1;
                }
                prev_sign = current_sign; // 更新前一个非零差的方向
            }
        }        
    }
}

// 计算到达方向切换点的时间最优轨迹函数
void PathTrackingPlanning::calculateTminTraj()
{
    Vector7d vm_tri; // 理想轨迹最大速度
    for (size_t i = 0; i < n_dof; i++) 
    {
        acc[i] = a_max[i];
        vm_tri[i] = std::sqrt(v0[i]*v0[i]/2.0 + acc[i]*q_delta[i]); // 计算理想的最大运动速度

        if (vm_tri[i] < v_max[i]) // 无匀速阶段
        {
            vm[i] = vm_tri[i];
            tf[i] = (2.0*vm[i] - v0[i]) / acc[i];
        } 
        else // 存在匀速阶段
        {
            vm[i] = v_max[i];
            tf[i] = q_delta[i]/vm[i] + (vm[i] - v0[i] + v0[i]*v0[i]/(2.0*vm[i])) / acc[i];
        }
        t1[i] = (vm[i]-v0[i])/acc[i];
        t2[i] = tf[i] - vm[i]/acc[i];
    }
}

// 计算到达路径点的时间函数并确定路径点同步运动时间 t_via_sync
void PathTrackingPlanning::calculateViaTimeANDdetermineSyncTime()
{
    Vector7d q_t1; // 加速阶段结束时刻位移
    Vector7d q_t2; // 匀速阶段结束时刻位移
    Vector7d t_via; // 到达路径点的时间

    bool all_joints_dec = true;
    bool any_joint_dec = false;

    for (size_t i = 0; i < n_dof; i++)
    {
        // 计算在到达方向转换点的梯形速度轨迹中关键位置(加速阶段结束时的位移 和 匀速阶段结束时的位移)
        q_t1[i] = 0.5 * (v0[i]+vm[i]) * t1[i]; // 加速阶段位移
        q_t2[i] = q_t1[i] + (t2[i]-t1[i]) * vm[i]; // 匀速阶段位移

        if (q_via_delta[i] <= q_t1[i]) // 相邻路径点在到达转向点的加速过程
        {
            t_via[i] = (std::sqrt(v0[i]*v0[i] + 2*acc[i]*q_via_delta[i])- v0[i]) / acc[i];
        } 
        else if (q_via_delta[i] > q_t1[i] && q_via_delta[i] <= q_t2[i]) // 相邻路径点在到达转向点的匀速过程
        {
            t_via[i] = (q_via_delta[i] - q_t1[i]) / vm[i] + t1[i];
        } 
        else // 相邻路径点在到达转向点的减速过程
        {
            t_via[i] = tf[i] - std::sqrt(2*(q_delta[i]-q_via_delta[i]) / acc[i]);
        }

        // 检查所有关节所处运动阶段
        if (t2[i] < 10e-3)  // 处于纯减速情况
        {
            any_joint_dec = true;
        } 
        else 
        {
            all_joints_dec = false;
        }
    }

    if (any_joint_dec && !all_joints_dec) // 只有部分关节处于减速阶段
    {
        for (size_t i = 0; i < n_dof; i++) 
        {
            if (t2[i] < 10e-3) 
            {
                t_via[i] = 0;
            }
        }
        t_via_sync = t_via.maxCoeff();
    } 
    else if (all_joints_dec) // 所有关节都处于减速阶段
    {
        t_via_sync = t_via.minCoeff();
    }
    else if (!any_joint_dec) // 所有关节都处于非减速阶段
    {
        t_via_sync = t_via.maxCoeff();
    }

    /* 确保所有关节能够在同步运动时间 t_via_sync 内到达路径点 */
    int t_via_found = 0;
    int i_increment = 0;
    double dt = 0.01;

    Vector7d q_via_max = Vector7d::Zero(7);
    while (!t_via_found && t_via_sync < 50)
    {
        t_via_found = 1;
        t_via_sync = t_via_sync + dt * i_increment;

        // 计算在路径点时间内可能移动的最大和最小距离
        for (size_t j = 0; j < n_dof; j++)
        {
            double t1_ = (v_max[j]-v0[j])/a_max[j];

            if (t1_ >= t_via_sync) // 仅有加速过程
            {
                q_via_max[j] = v0[j] * t_via_sync + 0.5 * a_max[j] * std::pow(t_via_sync, 2);  
            }
            else // 加速 + 匀速过程
            {
                q_via_max[j] = 0.5 * (v0[j]+v_max[j]) * t1_ + v_max[j] * (t_via_sync-t1_);
            }
            
            if (q_via_delta[j] > q_via_max[j]) // 在 t_via_sync 时间内关节无法到达路径点
            {
                t_via_found = 0;
            }
        }
        i_increment = i_increment + 1;
    }
    /* 确保所有关节能够在同步运动时间 t_via_sync 内到达路径点 */
}

// 调整路径点轨迹参数函数
void PathTrackingPlanning::adjustViaTVP()
{
    Vector7d vm_tri; // 理想轨迹最大速度
    for (size_t i = 0; i < n_dof; i++)
    {
        if ((i_via+1) == i_via_change_direction(i_change_direction[i],i)) // 如果下一路径点是方向切换点
        {
            if (q_delta[i] <= 0.5*v0[i]*t_via_sync) 
            {
                vm[i] = v0[i];
                t1[i] = 0;
                t2[i] = 0;
                tf[i] = 2*q_delta[i]/v0[i];
                acc[i] = vm[i]/tf[i]; // 加速度可能超限
            }
            else
            {   
                tf[i] = t_via_sync;
                vm_tri[i] = q_delta[i]/tf[i] + std::sqrt(std::pow((q_delta[i]/tf[i]-v0[i]/2), 2) + std::pow(v0[i], 2)/4);
                vm[i] = std::min(vm_tri[i], v_max[i]);
                acc[i] = (vm[i]*v0[i] - std::pow(vm[i], 2) - std::pow(v0[i], 2)/2) / (q_delta[i] - tf[i]*vm[i]);
                t1[i] = (vm[i] - v0[i]) / acc[i];
                t2[i] = tf[i] - vm[i]/acc[i]; 
                if (vm[i] - v0[i] < -10e-6)
                {
                    vm[i] = v0[i];
                    acc[i] = std::pow(v0[i], 2) / (2*q_delta[i]);
                    t1[i] = 0;
                    t2[i] = 0;
                    tf[i] = vm[i]/acc[i];
                }
            }
        }
        else // 如果下一路径点不是方向切换点
        {
            if (q_via_delta[i] < 0.5*v0[i]*t_via_sync) // 纯减速
            {
                vm[i] = v0[i];
                t1[i] = 0;
                t2[i] = 0;
                tf[i] = 2*q_via_delta[i]/v0[i];
                acc[i] = vm[i]/tf[i]; // 加速度可能超限
            } 
            else if ((q_via_delta[i] >= 0.5*v0[i]*t_via_sync) && (q_via_delta[i] < v0[i]*t_via_sync)) // 纯减速
            {
                vm[i] = v0[i];
                t1[i] = 0;
                t2[i] = 0;
                acc[i] = 2 * ((v0[i]*t_via_sync - q_via_delta[i])/(t_via_sync*t_via_sync));
                tf[i] = vm[i]/acc[i];
            }
            else if (q_via_delta[i] >= v0[i]*t_via_sync) // 加速 + 匀速
            {
                acc[i] = a_max[i];
                t1[i] = t_via_sync - std::sqrt(t_via_sync*t_via_sync - 2*(q_via_delta[i] - v0[i]*t_via_sync)/acc[i]);
                vm[i] = v0[i] + acc[i]*t1[i];
                t2[i] = t_via_sync + 1; // 虚拟时间
                tf[i] = t_via_sync + 1; // 虚拟时间
            }
        }
    }
}

// 更新参数函数
void PathTrackingPlanning::updateParameters()
{
    // 路径点参数更新
    for (size_t i = 0; i < n_dof; i++)
    {
        q_delta[i] = std::abs(q_f[i] - q_0[i]); // 到下一个方向变换点的绝对位移
        q_delta_sign[i] = determineSign(q_f[i] - q_0[i]); // 转向确定
        q_via_delta[i] = std::abs(q_via[i] - q_0[i]); // 相邻两个路径点之间的绝对位移
        v0[i] = std::abs(v_0[i]); // 计算轨迹的绝对速度
    }
}

/*
计算到路径点轨迹参数函数
    输出：
    1.加速度           a
    2.最大速度         vm
    3.加速阶段结束时刻  t1
    4.匀速阶段结束时刻  t2
    5.总运动时间       tf
    6.同步运动时间     t_via_sync
*/
void PathTrackingPlanning::calculateViaTraj()
{
    // 1.计算时间最优轨迹
    calculateTminTraj();

    // 2.计算到路径点的时间并确定同步运动时间
    calculateViaTimeANDdetermineSyncTime();

    // 3.调整路径点轨迹参数
    adjustViaTVP();
}

/*
计算轨迹数据函数
    输入：采样时间
    输出：
    1.轨迹位置 Vector7d q_current
    2.轨迹速度 Vector7d v_current
*/
void PathTrackingPlanning::calculateDesiredValues(double t)
{
    Vector7d q_delta_abs;  // 位移的绝对变化量
    Vector7d v_current_abs; // 速度的绝对值

    for (size_t i = 0; i < n_dof; i++)
    {
        if (q_via_delta[i] < kDeltaQMotionFinished) // 关节不进行运动
        {
            q_delta_abs[i] = 0.0; 
            v_current_abs[i] = 0.0;
        }
        else
        {
            if (t < t1[i]) // 加速阶段
            {
                q_delta_abs[i] = v0[i]*t + 0.5*acc[i]*t*t;
                v_current_abs[i] = v0[i] + acc[i]*t;
            } 
            else if (t >= t1[i] && t < t2[i]) // 处于匀速阶段
            {
                q_delta_abs[i] = 0.5*(v0[i]+vm[i])*t1[i] + vm[i]*(t-t1[i]);
                v_current_abs[i] = vm[i];
            } 
            else if (t >= t2[i] && t < tf[i]) // 处于减速阶段
            {
                q_delta_abs[i] = 0.5*(v0[i]+vm[i])*t1[i] + vm[i]*(t2[i]-t1[i]) + vm[i]*(t-t2[i]) - 0.5*acc[i]*(t-t2[i])*(t-t2[i]);
                v_current_abs[i] = vm[i] - acc[i]*(t-t2[i]);
            }
            else
            {
                q_delta_abs[i] = 0.5*(v0[i]+vm[i])*t1[i] + vm[i]*(t2[i]-t1[i]) + vm[i]*(tf[i]-t2[i]) - 0.5*acc[i]*(tf[i]-t2[i])*(tf[i]-t2[i]);
                v_current_abs[i] = 0.0; // 运动结束
            }
        }

        // 计算实际值
        q_current[i] = q_0[i] + q_delta_sign[i] * q_delta_abs[i]; // 计算当前关节位置
        v_current[i] = q_delta_sign[i] * v_current_abs[i]; // 计算当前关节速度
    }
}

// 初始化轨迹结构体函数
void PathTrackingPlanning::initializeTrajectory()
{
    trajectory.points.clear();  // 清空轨迹中的点
}

// 获取轨迹点数量的函数
size_t PathTrackingPlanning::getTrajectorySize()
{
    return trajectory.points.size();
}

// 运动轨迹生成函数
PathTrackingPlanning::Trajectory PathTrackingPlanning::startPlanning(double sample_time)
{
    /* 初始化规划数据 */
    PathTrackingPlanning::TrajectoryPoint point; // 轨迹点
    sample_time_ = sample_time; // 设置采样时间
    double time = 0.0; // 当前时间,用于生成轨迹

    // 存入轨迹初始位置构型和初速度
    for(size_t i = 0; i < 7; ++i)
    {
        point.position[i] = init_position[i];
        point.velocity[i] = 0.0;
    }
    trajectory.points.push_back(point);

    // 路径点遍历，开始规划
    while (!motion_finished)
    {
        // 1.更新参数
        updateParameters();

        // 2.计算到达下一路径点的梯形轨迹曲线参数(v0, acc, vm, t1, t2, tf 和 t_via_sync)
        calculateViaTraj();

        /***********************调试代码**************************/
        // std::cout << "i_via : " << i_via << ", t_via_sync : " << t_via_sync << std::endl;
        // std::cout << "q_0 : " << q_0 << std::endl;
        // std::cout << "v_0 : " << v_0 << std::endl;
        // std::cout << "q_via : " << q_via << std::endl;
        // std::cout << "q_f : " << q_f << std::endl;
        // std::cout << "q_via_delta : " << q_via_delta << std::endl;
        // std::cout << "q_delta : " << q_delta << std::endl;
        // std::cout << "v0 : " << v0 << std::endl;
        // std::cout << "acc : " << acc << std::endl;
        // std::cout << "vm : " << vm << std::endl;
        // std::cout << "t1 : " << t1 << std::endl;
        // std::cout << "t2 : " << t2 << std::endl;
        // std::cout << "tf : " << tf << std::endl;
        // std::cout << "----------------------------------------" << std::endl;
        /***********************调试代码**************************/

        // 3.生成到达下一路径点的运动轨迹
        while ((time + sample_time_) <= t_via_sync)
        {
            // 更新当前时间
            time += sample_time_;

            // 计算当前时间对应的期望位移
            calculateDesiredValues(time);


            /***********************调试代码**************************/
            // for (size_t i = 0; i < n_dof; ++i)
            // {
            //     if (std::isnan(q_current[i]) || std::isinf(q_current[i]))
            //     {
            //         std::cout << "关节 " << i+1 << " 的位置计算出现错误: " << std::endl;
            //         std::cout << "当前路径点索引为 : " << i_via << std::endl;
            //         std::cout << "t_via_sync : " << t_via_sync << ", v0 : " << v0[i] << ", acc : " << acc[i] << ", vm : " << vm[i] 
            //         << ", t1 : " << t1[i] << ", t2 : " << t2[i] << ", tf : " << tf[i] 
            //         << ", q_via_delta : " << q_via_delta[i] << ", q_delta : " << q_delta[i] 
            //         << ", q_0 : " << q_0[i] << ", q_via : " << q_via[i] << ", q_f : " << q_f[i] << std::endl;

            //         throw std::runtime_error("位置计算出现 NaN 或 Inf 错误，程序终止。");
            //     }
            // }
            /***********************调试代码**************************/


            // 存储轨迹点
            for (size_t j = 0; j < n_dof; ++j)
            {
                point.position[j] = q_current[j];
                point.velocity[j] = v_current[j];
            }
            trajectory.points.push_back(point);
        }

        // 4.判断运动规划是否结束并更新状态
        if (i_via == (static_cast<int>(n_via) - 2)) // 运动规划结束
        {
            time = 0.0; // 重置时间
            motion_finished = true; // 设置运动规划结束标志
            t_f_sync += t_via_sync; // 更新总运动时间
        }
        else
        {
            time = 0.0; // 重置时间
            v_0 = v_current; // 更新初始速度为当前速度
            q_0 = q_current; // 更新初始位置为当前位置
            i_via++; // 更新路径点索引
            q_via = q_path.row(i_via + 1); // 更新下一个路径点位置
            for (size_t j = 0; j < n_dof; j++)
            {
                // 判断到达的路径点是否是转向点
                if (i_via == i_via_change_direction(i_change_direction[j],j))
                {
                    i_change_direction[j] += 1;
                    q_f[j] = q_path(i_via_change_direction(i_change_direction[j],j),j); // 将下一个转向点作为目标点
                }
            }
            t_f_sync += t_via_sync; // 更新总运动时间
        }
    }

    // 梯形速度曲线参数重置
    acc.setZero();
    vm.setZero();
    t1.setZero();
    t2.setZero();
    tf.setZero();

    return trajectory; // 返回轨迹数据
}

// 绘制轨迹曲线图函数
void PathTrackingPlanning::plotTrajectory(const Trajectory& trajectory, 
                                        const std::string& data_dir, 
                                        const std::string& file_name)
{
   // 确保目录存在
    std::string command = "mkdir -p " + data_dir;
    std::system(command.c_str());

    // 定义数据文件名和图像文件名
    std::string temp_filename = data_dir + "/" + file_name + ".dat";
    std::string plot_filename = data_dir + "/" + file_name + ".png";

    // 打开临时文件保存绘图数据
    std::ofstream temp_file(temp_filename);
    if (temp_file.is_open())
    {
        // 将轨迹数据保存到文件，添加时间数据作为第一列
        double time_step = sample_time_;  
        double current_time = 0.0;

        for (const auto& point : trajectory.points)
        {
            // 写入时间
            temp_file << current_time << " ";

            // 写入位置数据（7个关节）
            for (size_t i = 0; i < 7; ++i)
            {
                temp_file << point.position[i] << " ";
            }

            // 写入速度数据（7个关节）
            for (size_t i = 0; i < 7; ++i)
            {
                temp_file << point.velocity[i] << " ";
            }

            temp_file << "\n";

            // 增加时间
            current_time += time_step;
        }
        temp_file.close();

        // 使用 gnuplot 绘制位置和速度（上下排列的 multiplot）
        std::string plot_command = "gnuplot -e \"set terminal png size 800,600; set output '" + plot_filename + "'; "
                                    "set multiplot layout 2,1 title 'Trajectory Plot'; "
                                    "set xlabel 'Time (s)'; set ylabel 'Position (radians)'; "
                                    "plot '" + temp_filename + "' using 1:2 with lines title 'Position 1', "
                                    "'" + temp_filename + "' using 1:3 with lines title 'Position 2', "
                                    "'" + temp_filename + "' using 1:4 with lines title 'Position 3', "
                                    "'" + temp_filename + "' using 1:5 with lines title 'Position 4', "
                                    "'" + temp_filename + "' using 1:6 with lines title 'Position 5', "
                                    "'" + temp_filename + "' using 1:7 with lines title 'Position 6', "
                                    "'" + temp_filename + "' using 1:8 with lines title 'Position 7'; "
                                    "set ylabel 'Velocity (radians/s)'; "
                                    "plot '" + temp_filename + "' using 1:9 with lines title 'Velocity 1', "
                                    "'" + temp_filename + "' using 1:10 with lines title 'Velocity 2', "
                                    "'" + temp_filename + "' using 1:11 with lines title 'Velocity 3', "
                                    "'" + temp_filename + "' using 1:12 with lines title 'Velocity 4', "
                                    "'" + temp_filename + "' using 1:13 with lines title 'Velocity 5', "
                                    "'" + temp_filename + "' using 1:14 with lines title 'Velocity 6', "
                                    "'" + temp_filename + "' using 1:15 with lines title 'Velocity 7'; "
                                    "unset multiplot\"";

        std::system(plot_command.c_str());

        std::cout << "绘图完成，图像保存为 '" << plot_filename << "'" << std::endl;
    }
    else
    {
        std::cerr << "无法打开临时文件 " << temp_filename << std::endl;
    }
}

// 保存轨迹数据函数
void PathTrackingPlanning::saveTrajectoryData(std::string position_file_path, std::string velocity_file_path)
{
    // 打开文件用于写入位置数据
    std::ofstream position_file(position_file_path);
    if (!position_file.is_open()) 
    {
        std::cerr << "无法打开位置数据文件 " << position_file_path << std::endl;
        return;
    }

    // 打开文件用于写入速度数据
    std::ofstream velocity_file(velocity_file_path);
    if (!velocity_file.is_open()) 
    {
        std::cerr << "无法打开速度数据文件 " << velocity_file_path << std::endl;
        return;
    }

    // 遍历轨迹中的所有点，将位置和速度分别写入到对应的文件中
    for (const auto& point : trajectory.points)
    {
        // 写入位置数据到 position_file
        for (size_t i = 0; i < 7; ++i) 
        {
            position_file << point.position[i] << " ";  // 写入位置
        }
        position_file << std::endl;  

        // 写入速度数据到 velocity_file
        for (size_t i = 0; i < 7; ++i) 
        {
            velocity_file << point.velocity[i] << " ";  // 写入速度
        }
        velocity_file << std::endl; 
    }

    // 关闭文件
    position_file.close();
    velocity_file.close();

    std::cout << "轨迹数据已保存! " << std::endl;
}
