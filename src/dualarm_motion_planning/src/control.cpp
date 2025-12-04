#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "dualarm_motion_planning/srv/traj_para.hpp" 

#include <sstream>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <cmath>
#include <vector>
#include <chrono> 

#include "path_tracking_planning.h"
#include <ament_index_cpp/get_package_share_directory.hpp>

// 如果没有在 CMake 中定义 PACKAGE_SOURCE_DIR，则使用安装路径
#ifndef PACKAGE_SOURCE_DIR
#define PACKAGE_SOURCE_DIR ""
#endif

using Traj = PathTrackingPlanning::Trajectory;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandleFollowJointTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

class ControlNode : public rclcpp::Node
{
public:
    ControlNode() : Node("control_node")
    {
        // 1. 声明并获取参数
        this->declare_parameter<double>("sample_time", 0.01);
        this->declare_parameter<int>("n_dof", 7);
        this->declare_parameter<double>("joint_tolerance", 1e-4);
        this->declare_parameter<int>("robot_id", 1);
        this->declare_parameter<std::string>("action_name", "joint_trajectory_controller/follow_joint_trajectory");
        this->declare_parameter<std::vector<std::string>>("joint_names", std::vector<std::string>({}));

        this->get_parameter("sample_time", sample_time_);
        this->get_parameter("n_dof", n_dof_);
        this->get_parameter("joint_tolerance", joint_tolerance_);
        this->get_parameter("robot_id", robot_id_);
        this->get_parameter("action_name", action_name_);
        this->get_parameter("joint_names", joint_names_);

        if ((int)joint_names_.size() != n_dof_)
        {
            RCLCPP_WARN(this->get_logger(), "读取到的 joint_names 数量与 n_dof 不匹配 (%lu vs %d).", joint_names_.size(), n_dof_);
        }

        // 初始化变量
        v_0_ = Eigen::VectorXd::Zero(n_dof_);

        // 确定包路径
        std::string source_dir = PACKAGE_SOURCE_DIR;
        if (!source_dir.empty()) {
            package_path_ = source_dir;
            RCLCPP_INFO(this->get_logger(), "Using source directory for data: %s", package_path_.c_str());
        } else {
            try {
                package_path_ = ament_index_cpp::get_package_share_directory("dualarm_motion_planning");
                RCLCPP_INFO(this->get_logger(), "Using install directory for data: %s", package_path_.c_str());
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Failed to locate package: %s", e.what());
                package_path_ = ".";
            }
        }

        // 2. 初始化 Action Client
        action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
            this, action_name_);

        // 3. 初始化 Service Server
        std::string service_name = "traj_parameter_r" + std::to_string(robot_id_);
        service_server_ = this->create_service<dualarm_motion_planning::srv::TrajPara>(
            service_name,
            std::bind(&ControlNode::start_move, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "控制节点启动 (Robot ID: %d), Service: %s, Action: %s", 
                    robot_id_, service_name.c_str(), action_name_.c_str());
    }

private:
    // 参数变量
    double sample_time_;
    int n_dof_;
    double joint_tolerance_;
    int robot_id_;
    std::string action_name_;
    std::vector<std::string> joint_names_;
    std::string package_path_;

    // 状态变量
    double delay_time_ = 0.0;
    double v_max_ = 0.0;
    double a_max_ = 0.0;
    Eigen::VectorXd v_0_;
    Eigen::MatrixXd q_path_;
    Traj traj_;

    // ROS 2 接口
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;
    rclcpp::Service<dualarm_motion_planning::srv::TrajPara>::SharedPtr service_server_;

    // 服务回调函数
    void start_move(const std::shared_ptr<dualarm_motion_planning::srv::TrajPara::Request> req,
                    std::shared_ptr<dualarm_motion_planning::srv::TrajPara::Response> res)
    {
        bool change_goal = false;
        int n_via = req->qgoal.size() / n_dof_;
        
        if (n_via <= 0)
        {
            RCLCPP_ERROR(this->get_logger(), "start_move: invalid via points");
            res->received = false;
            return;
        }

        q_path_.resize(n_via, n_dof_);
        q_path_.setZero();

        // 判断目标是否发生变化
        for (int i = 0; i < n_via; ++i)
        {
            for (int j = 0; j < n_dof_; ++j)
            {
                double val = req->qgoal[i * n_dof_ + j];
                // 使用简单的容差比较浮点数
                if (std::abs(q_path_(i, j) - val) > 1e-6) {
                    change_goal = true;
                }
                q_path_(i, j) = val;
            }
        }
        if (std::abs(v_max_ - req->v_max) > 1e-6 || std::abs(a_max_ - req->a_max) > 1e-6) {
            change_goal = true;
        }

        if (change_goal)
        {
            delay_time_ = req->delaytime;
            v_max_ = req->v_max;
            a_max_ = req->a_max;

            // 生成轨迹
            PathTrackingPlanning planner(v_max_, a_max_, v_0_, q_path_);
            traj_ = planner.startPlanning(sample_time_);
            size_t num_points = planner.getTrajectorySize();

            // 绘图保存
            planner.plotTrajectory(traj_, package_path_ + "/data", "r" + std::to_string(robot_id_) + "_trajectory");

            // 检查规划轨迹是否到达目标位置
            for (int i = 0; i < n_dof_; ++i)
            {
                double desired = q_path_(n_via - 1, i);
                double actual = traj_.points[num_points - 1].position[i];
                if (std::abs(actual - desired) > joint_tolerance_)
                {
                    RCLCPP_WARN(this->get_logger(), "关节 %d 未到达目标位置: desired=%f actual=%f", i + 1, desired, actual);
                    break;
                }
            }
            
            // 发送轨迹
            executeTrajectory(traj_, joint_names_, delay_time_);

            res->received = true;
        }
        else 
        {
            res->received = false;
        }
    }

    // 执行轨迹函数
    void executeTrajectory(const Traj& trajectory,
                           const std::vector<std::string>& joint_names_in,
                           double delay_time_in)
    {
        if (joint_names_in.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "executeTrajectory: joint_names is empty.");
            return;
        }

        if (!action_client_->wait_for_action_server(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(this->get_logger(), "Action server %s not available.", action_name_.c_str());
            return;
        }

        FollowJointTrajectory::Goal goal;
        goal.trajectory.joint_names = joint_names_in;

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions.resize(n_dof_);
        point.velocities.resize(n_dof_);

        for (size_t idx = 0; idx < trajectory.points.size(); ++idx)
        {
            for (int i = 0; i < n_dof_; ++i)
            {
                point.positions[i] = trajectory.points[idx].position[i];
                point.velocities[i] = trajectory.points[idx].velocity[i];
            }
            // ROS 2 Duration 使用秒和纳秒构造，或者直接用 from_seconds
            point.time_from_start = rclcpp::Duration::from_seconds(idx * sample_time_);
            goal.trajectory.points.push_back(point);
        }

        // 延时处理
        if (delay_time_in > 0) {
            // 使用 duration_cast 将 double 秒显式转换为 nanoseconds 整数
            rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(delay_time_in)));
        }

        // 发送 Goal
        RCLCPP_INFO(this->get_logger(), "Sending goal with %zu points...", goal.trajectory.points.size());
        
        auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
        
        // 绑定回调（可选，简单起见这里只打印结果）
        send_goal_options.result_callback = 
            [this](const GoalHandleFollowJointTrajectory::WrappedResult & result) {
                switch (result.code) {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        RCLCPP_INFO(this->get_logger(), "Trajectory execution succeeded");
                        break;
                    case rclcpp_action::ResultCode::ABORTED:
                        RCLCPP_ERROR(this->get_logger(), "Trajectory execution was aborted");
                        break;
                    case rclcpp_action::ResultCode::CANCELED:
                        RCLCPP_ERROR(this->get_logger(), "Trajectory execution was canceled");
                        break;
                    default:
                        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                        break;
                }
            };
            
        action_client_->async_send_goal(goal, send_goal_options);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}