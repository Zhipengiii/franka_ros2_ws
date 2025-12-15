#include <thread>
#include <cstdlib>
#include <chrono>
#include <filesystem>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include "dualarm_motion_planning/srv/give_goal.hpp"
#include "dualarm_motion_planning/srv/traj_para.hpp"

#include "path_tracking_planning.h"
#include "collision_detection.h"
#include "find_point_on_line.h"
#include "dualarm_coordinator.h"
#include <ament_index_cpp/get_package_share_directory.hpp>

// 宏定义处理包路径
#ifndef PACKAGE_SOURCE_DIR
#define PACKAGE_SOURCE_DIR ""
#endif

using namespace std::chrono_literals;

// 定义生成碰撞检测样本的类型
enum SampleType
{
    DUAL_ARM,    // 双臂笛卡尔空间采样点
    LEFT_ARM,    // 左臂笛卡尔空间采样点
    RIGHT_ARM    // 右臂笛卡尔空间采样点
};

class CoordNode : public rclcpp::Node
{
public:
    // 构造函数
    CoordNode() : Node("coord_node")
    {
        // 1. 声明参数
        this->declare_parameter<int>("offline_replanning_mode", 1);
        this->declare_parameter<double>("collision_threshold", 0.05);
        this->declare_parameter<int>("n_dof", 7);
        this->declare_parameter<double>("collision_detection_sample_time", 0.01);
        this->declare_parameter<std::vector<int64_t>>("robot_id_pair", {1, 2});
        this->declare_parameter<std::string>("r1_group_name", "left_arm");
        this->declare_parameter<std::string>("r2_group_name", "right_arm");
        this->declare_parameter<std::vector<std::string>>("r1_arm_links", std::vector<std::string>({}));
        this->declare_parameter<std::vector<std::string>>("r2_arm_links", std::vector<std::string>({}));

        // 2. 获取参数
        this->get_parameter("offline_replanning_mode", offline_replanning_mode);
        this->get_parameter("collision_threshold", collision_threshold);
        this->get_parameter("n_dof", n_dof);
        this->get_parameter("collision_detection_sample_time", collision_detection_sample_time);
        this->get_parameter("r1_group_name", r1_group_name);
        this->get_parameter("r2_group_name", r2_group_name);
        this->get_parameter("r1_arm_links", r1_links);
        this->get_parameter("r2_arm_links", r2_links);
        std::vector<int64_t> id_pair;
        this->get_parameter("robot_id_pair", id_pair);
        if(id_pair.size() == 2) {
            robot_id_pair = {static_cast<int>(id_pair[0]), static_cast<int>(id_pair[1])};
        }

        // 3. 确定包路径
        std::string source_dir = PACKAGE_SOURCE_DIR;
        if (!source_dir.empty()) {
            package_path = source_dir;
            RCLCPP_INFO(this->get_logger(), "Using source directory for data: %s", package_path.c_str());
        } else {
            try {
                package_path = ament_index_cpp::get_package_share_directory("dualarm_motion_planning");
                RCLCPP_INFO(this->get_logger(), "Using install directory for data: %s", package_path.c_str());
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Failed to locate package: %s", e.what());
                package_path = ".";
            }
        }
        // 创建 data 目录
        std::string data_dir = package_path + "/data";
        if (!std::filesystem::exists(data_dir)) {
            std::filesystem::create_directories(data_dir);
        }

        // 4. 初始化服务
        std::string service_name = "give_goal_" + std::to_string(robot_id_pair[0]) + "_" + std::to_string(robot_id_pair[1]);
        service_getgoal_ = this->create_service<dualarm_motion_planning::srv::GiveGoal>(
            service_name,
            std::bind(&CoordNode::get_goal, this, std::placeholders::_1, std::placeholders::_2));
        
        // 初始化客户端
        left_traj_client_ = this->create_client<dualarm_motion_planning::srv::TrajPara>(
            "traj_parameter_r" + std::to_string(robot_id_pair[0]));
        right_traj_client_ = this->create_client<dualarm_motion_planning::srv::TrajPara>(
            "traj_parameter_r" + std::to_string(robot_id_pair[1]));

        // 初始化变量
        v_0_r1 = Eigen::VectorXd::Zero(n_dof);
        v_0_r2 = Eigen::VectorXd::Zero(n_dof);
    }

    // 初始化模型，必须在 shared_from_this() 可用后调用
    void init_model()
    {
        robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(shared_from_this(), "robot_description");
        kinematic_model_ = robot_model_loader_->getModel();
        kinematic_state_ = std::make_shared<moveit::core::RobotState>(kinematic_model_);
        kinematic_state_->setToDefaultValues();
        joint_model_group_left_ = kinematic_model_->getJointModelGroup(r1_group_name);
        joint_model_group_right_ = kinematic_model_->getJointModelGroup(r2_group_name);
        RCLCPP_INFO(this->get_logger(), "机器人模型初始化完成.");
    }

    void run()
    {
        // 确保模型已初始化
        if (!kinematic_state_) init_model();

        RCLCPP_INFO(this->get_logger(), "采用延时模式进行协调.等待目标中...");
        rclcpp::Rate loop_rate(10); 

        while (rclcpp::ok()) 
        {
            if (robotid_r1 && robotid_r2 && (change_goal_r1 || change_goal_r2))
            {
                // 生成双臂碰撞检测样本
                generate_collision_samples(DUAL_ARM);

                // 获取碰撞检测样本的最大值
                size_t sample_count_r1 = position_cartesian_r1.cols(); 
                size_t sample_count_r2 = position_cartesian_r2.cols(); 
                sample_size_max = std::max(sample_count_r1, sample_count_r2);

                // 初始化协调器 (注意传入 shared_from_this())
                DualArmCoordinator coordinator(shared_from_this(), collision_threshold, sample_size_max, collision_detection_sample_time
                                            , r1_links, r2_links, r1_group_name, r2_group_name);
                coordinator.coordinate(position_cartesian_r1, position_cartesian_r2, trajectory_r1, trajectory_r2);

                if (coordinator.diag_collision_flag)
                {
                    // 发生碰撞，但未阻塞
                    if (!coordinator.deadlock_flag)
                    {
                        coordinator.saveCollisionMatrix(package_path + "/data/collision_matrix_non_blocking.txt");
                        update_coord_result(coordinator);
                        if (offline_replanning_mode == 1) printCoordinationResult();
                        sendTrajectoryParameters();
                    }
                    else // 发生阻塞
                    {
                        coordinator.saveCollisionMatrix(package_path + "/data/collision_matrix_blocking.txt");

                        if (coordinator.blocking_conflict_r == DualArmCoordinator::CONFLICT_R1) // R1冲突，对其进行重规划
                        {
                            process_replanning(coordinator, LEFT_ARM);
                        }
                        else if (coordinator.blocking_conflict_r == DualArmCoordinator::CONFLICT_R2)
                        {
                            process_replanning(coordinator, RIGHT_ARM);
                        }
                    }
                }
                else 
                {
                    if (offline_replanning_mode == 1) printCoordinationResult();
                    sendTrajectoryParameters();
                }

                // 参数重置
                reset_parameters();
            }
            loop_rate.sleep();
        }
    }

private:
    // 成员变量
    std::string package_path;
    int offline_replanning_mode;
    int n_via_left = 3;
    int n_via_right = 3;
    int n_dof;
    double collision_threshold;
    double collision_detection_sample_time;
    std::vector<std::string> r1_links;
    std::vector<std::string> r2_links;
    std::string r1_group_name;
    std::string r2_group_name;
    std::vector<int> robot_id_pair;

    int robotid_r1 = 0;
    int robotid_r2 = 0;
    double delay_time_r1 = 0;
    double delay_time_r2 = 0;
    double v_max_r1 = 1;
    double v_max_r2 = 1;
    double a_max_r1 = 1;
    double a_max_r2 = 1;
    
    Eigen::VectorXd v_0_r1;
    Eigen::VectorXd v_0_r2;
    Eigen::MatrixXd q_path_r1;
    Eigen::MatrixXd q_path_r2;

    bool change_goal_r1 = false;
    bool change_goal_r2 = false;
    bool prior_r1 = false;
    bool prior_r2 = false;
    int sample_size_max;
    
    Eigen::MatrixXd position_cartesian_r1;
    Eigen::MatrixXd position_cartesian_r2;
    PathTrackingPlanning::Trajectory trajectory_r1;
    PathTrackingPlanning::Trajectory trajectory_r2;

    // ROS 2 接口
    rclcpp::Service<dualarm_motion_planning::srv::GiveGoal>::SharedPtr service_getgoal_;
    rclcpp::Client<dualarm_motion_planning::srv::TrajPara>::SharedPtr left_traj_client_;
    rclcpp::Client<dualarm_motion_planning::srv::TrajPara>::SharedPtr right_traj_client_;

    // MoveIt 接口
    std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
    moveit::core::RobotModelPtr kinematic_model_;
    moveit::core::RobotStatePtr kinematic_state_;
    const moveit::core::JointModelGroup* joint_model_group_left_;
    const moveit::core::JointModelGroup* joint_model_group_right_;

    // 回调函数
    void get_goal(const std::shared_ptr<dualarm_motion_planning::srv::GiveGoal::Request> req,
                  std::shared_ptr<dualarm_motion_planning::srv::GiveGoal::Response> res)
    {
        if (req->robotid == robot_id_pair[0])
        {
            res->received = 1;
            n_via_left = req->qgoal.size() / n_dof;
            q_path_r1.resize(n_via_left, n_dof);
            robotid_r1 = req->robotid;
            v_max_r1 = req->v_max;
            a_max_r1 = req->a_max;

            for (int i = 0; i < n_via_left; ++i)
                for (int j = 0; j < n_dof; ++j)
                    q_path_r1(i, j) = req->qgoal[i*n_dof + j];

            change_goal_r1 = true;
        }

        if (req->robotid == robot_id_pair[1])
        {
            res->received = 1;  
            n_via_right = req->qgoal.size() / n_dof;
            q_path_r2.resize(n_via_right, n_dof);  
            robotid_r2 = req->robotid;
            v_max_r2 = req->v_max;
            a_max_r2 = req->a_max;

            for (int i = 0; i < n_via_right; ++i)
                for (int j = 0; j < n_dof; ++j)
                    q_path_r2(i, j) = req->qgoal[i*n_dof + j];

            change_goal_r2 = true;
        }
    }

    void generate_collision_samples(SampleType sample_type)
    {
        if (sample_type == DUAL_ARM || sample_type == LEFT_ARM)
        {
            trajectory_r1.points.clear(); 
            PathTrackingPlanning path_tracking_planning_r1(v_max_r1, a_max_r1, v_0_r1, q_path_r1); 
            trajectory_r1 = path_tracking_planning_r1.startPlanning(collision_detection_sample_time);
            size_t sample_count_r1 = path_tracking_planning_r1.getTrajectorySize();
            
            // 路径文件名根据类型区分
            std::string fname = (sample_type == LEFT_ARM) ? "left_sample_trajectory_replanning" : "left_sample_trajectory";
            path_tracking_planning_r1.plotTrajectory(trajectory_r1, package_path+"/data", fname);

            size_t r1_links_size = r1_links.size();
            position_cartesian_r1.resize(3*r1_links_size, sample_count_r1);
            
            std::vector<double> joint_values(n_dof);
            for (size_t j = 0; j < sample_count_r1; ++j)
            {
                for (int i = 0; i < n_dof; ++i) joint_values[i] = trajectory_r1.points[j].position[i];
                kinematic_state_->setJointGroupPositions(joint_model_group_left_, joint_values); 

                for (size_t m = 0; m < r1_links_size; ++m)
                {
                    Eigen::Isometry3d tf = kinematic_state_->getGlobalLinkTransform(r1_links[m]);
                    position_cartesian_r1(3*m, j) = tf.translation()[0];
                    position_cartesian_r1(3*m+1, j) = tf.translation()[1];
                    position_cartesian_r1(3*m+2, j) = tf.translation()[2];
                }
            }
        }

        if (sample_type == DUAL_ARM || sample_type == RIGHT_ARM)
        {
            trajectory_r2.points.clear(); 
            PathTrackingPlanning path_tracking_planning_r2(v_max_r2, a_max_r2, v_0_r2, q_path_r2); 
            trajectory_r2 = path_tracking_planning_r2.startPlanning(collision_detection_sample_time);
            size_t sample_count_r2 = path_tracking_planning_r2.getTrajectorySize();

            std::string fname = (sample_type == RIGHT_ARM) ? "right_sample_trajectory_replanning" : "right_sample_trajectory";
            path_tracking_planning_r2.plotTrajectory(trajectory_r2, package_path+"/data", fname);

            size_t r2_links_size = r2_links.size();
            position_cartesian_r2.resize(3*r2_links_size, sample_count_r2);

            std::vector<double> joint_values(n_dof);
            for (size_t j = 0; j < sample_count_r2; ++j)
            {
                for (int i = 0; i < n_dof; ++i) joint_values[i] = trajectory_r2.points[j].position[i];
                kinematic_state_->setJointGroupPositions(joint_model_group_right_, joint_values); 

                for (size_t m = 0; m < r2_links_size; ++m)
                {
                    Eigen::Isometry3d tf = kinematic_state_->getGlobalLinkTransform(r2_links[m]);
                    position_cartesian_r2(3*m, j) = tf.translation()[0];
                    position_cartesian_r2(3*m+1, j) = tf.translation()[1];
                    position_cartesian_r2(3*m+2, j) = tf.translation()[2];
                }
            }
        }
    }

    void update_coord_result(const DualArmCoordinator& coordinator)
    {
        prior_r1 = coordinator.prior_r1_;
        prior_r2 = coordinator.prior_r2_;
        delay_time_r1 = coordinator.delay_time_r1_;
        delay_time_r2 = coordinator.delay_time_r2_;
    }

    void process_replanning(DualArmCoordinator& coordinator, SampleType conflict_arm_type)
    {
        if (conflict_arm_type == LEFT_ARM)
        {
            n_via_left = coordinator.plan.trajectory_.joint_trajectory.points.size();
            q_path_r1.resize(n_via_left, n_dof);
            for (int i = 0; i < n_via_left; ++i)
                for (int j = 0; j < n_dof; ++j)
                    q_path_r1(i, j) = coordinator.plan.trajectory_.joint_trajectory.points[i].positions[j];

            generate_collision_samples(LEFT_ARM);
        }
        else
        {
            n_via_right = coordinator.plan.trajectory_.joint_trajectory.points.size();
            q_path_r2.resize(n_via_right, n_dof);
            for (int i = 0; i < n_via_right; ++i)
                for (int j = 0; j < n_dof; ++j)
                    q_path_r2(i, j) = coordinator.plan.trajectory_.joint_trajectory.points[i].positions[j];

            generate_collision_samples(RIGHT_ARM);
        }

        // 重新协调
        size_t sample_count_r1 = position_cartesian_r1.cols();
        size_t sample_count_r2 = position_cartesian_r2.cols();
        sample_size_max = std::max(sample_count_r1, sample_count_r2);
        
        DualArmCoordinator new_coordinator(shared_from_this(), collision_threshold, sample_size_max, collision_detection_sample_time
                                          , r1_links, r2_links, r1_group_name, r2_group_name);
        new_coordinator.coordinate(position_cartesian_r1, position_cartesian_r2, trajectory_r1, trajectory_r2);

        std::string re_co_matrix_path = (conflict_arm_type == LEFT_ARM) ? "/data/collision_matrix_replanning_r1.txt" 
                                                                        : "/data/collision_matrix_replanning_r2.txt";
        new_coordinator.saveCollisionMatrix(package_path + re_co_matrix_path);

        if (!new_coordinator.deadlock_flag)
        {
            update_coord_result(new_coordinator);
            if (offline_replanning_mode == 1) printCoordinationResult();
            sendTrajectoryParameters();
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "重规划后依然发生死锁阻塞，无法协调!");
        }
    }

    // 发送轨迹参数到各臂的轨迹跟踪节点
    void sendTrajectoryParameters()
    {
        auto req1 = std::make_shared<dualarm_motion_planning::srv::TrajPara::Request>();
        auto req2 = std::make_shared<dualarm_motion_planning::srv::TrajPara::Request>();

        // 填充参数
        req1->robotid = robotid_r1;
        req1->v_max = v_max_r1;
        req1->a_max = a_max_r1;
        req1->delaytime = delay_time_r1;
        req1->qgoal.resize(n_via_left * n_dof);
        for (int i = 0; i < n_via_left; ++i)
            for (int j = 0; j < n_dof; ++j)
                req1->qgoal[i * n_dof + j] = q_path_r1(i, j);

        req2->robotid = robotid_r2;
        req2->v_max = v_max_r2;
        req2->a_max = a_max_r2;
        req2->delaytime = delay_time_r2;
        req2->qgoal.resize(n_via_right * n_dof);
        for (int i = 0; i < n_via_right; ++i)
            for (int j = 0; j < n_dof; ++j)
                req2->qgoal[i * n_dof + j] = q_path_r2(i, j);

        // 异步发送
        auto future1 = left_traj_client_->async_send_request(req1);
        auto future2 = right_traj_client_->async_send_request(req2);

        if (future1.wait_for(3s) == std::future_status::timeout || 
            future2.wait_for(3s) == std::future_status::timeout)
        {
            RCLCPP_WARN(this->get_logger(), "服务调用超时");
        }
    }

    void reset_parameters()
    {
        delay_time_r1 = 0;
        delay_time_r2 = 0;
        change_goal_r1 = false;
        change_goal_r2 = false;
        robotid_r1 = 0;
        robotid_r2 = 0;
        prior_r1 = false;
        prior_r2 = false;
    }

    void printCoordinationResult()
    {
        if (delay_time_r1 == 0 && delay_time_r2 == 0)
        {
            RCLCPP_INFO(this->get_logger(), "双臂无干涉，未进行协调.");
        }
        else if (delay_time_r1 >= delay_time_r2)
        {
            delay_time_r2 = 0; // 优化输出
            RCLCPP_INFO(this->get_logger(), "双臂运动路径干涉，进行协调...");
            RCLCPP_INFO(this->get_logger(), "延迟左臂，延迟时间为 : %.3f 秒", delay_time_r1);
        }
        else
        {
            delay_time_r1 = 0; // 优化输出
            RCLCPP_INFO(this->get_logger(), "双臂运动路径干涉，进行协调...");
            RCLCPP_INFO(this->get_logger(), "延迟右臂，延迟时间为 : %.3f 秒", delay_time_r2);
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CoordNode>();

    // 使用多线程执行器，一个线程跑 spin 处理服务回调，主线程跑 run 逻辑
    rclcpp::executors::MultiThreadedExecutor executor;

    // 主线程
    executor.add_node(node);
    
    // 处理回调的线程
    std::thread spinner_thread([&executor](){
        executor.spin();
    });

    // 运行主逻辑
    node->run();

    executor.cancel();
    spinner_thread.join();
    rclcpp::shutdown();
    return 0;
}