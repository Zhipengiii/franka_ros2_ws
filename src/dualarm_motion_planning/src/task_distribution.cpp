#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include "dualarm_motion_planning/srv/give_goal.hpp"
#include "dualarm_motion_planning/msg/task_action.hpp"
#include "dualarm_motion_planning/msg/task_action_list.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <thread>
#include <algorithm>
#include <vector>
#include <string>
#include <mutex>
#include <cmath>

#define PI 3.14159265358979323846

// 宏定义处理包路径
#ifndef PACKAGE_SOURCE_DIR
#define PACKAGE_SOURCE_DIR ""
#endif

// 定义动作类型枚举
enum ActionType 
{
    HAND_ACTION = 1,      // 手动作
    JOINT_SPACE = 2,      // 关节空间位置
    CARTESIAN_SPACE = 3   // 笛卡尔空间位置
};

// 定义任务结构体
struct Task 
{
    std::string action_name; // 动作名称
    std::vector<double> goal_data;  // 目标数据
    double v_max; 
    double a_max; 
    ActionType action_type;  
};

class TaskDistributionNode : public rclcpp::Node
{
public:
    TaskDistributionNode() : Node("dual_arm_task_dispatcher")
    {
        // 1. 声明并获取参数
        this->declare_parameter<double>("v_max", 1.0);
        this->declare_parameter<double>("a_max", 1.0);
        this->declare_parameter<int>("n_dof", 7);
        this->declare_parameter<double>("joint_tolerance", 0.0001);
        this->declare_parameter<std::vector<std::string>>("left_joint_names", std::vector<std::string>());
        this->declare_parameter<std::vector<std::string>>("right_joint_names", std::vector<std::string>());

        this->get_parameter("v_max", v_max_);
        this->get_parameter("a_max", a_max_);
        this->get_parameter("n_dof", n_dof_);
        this->get_parameter("joint_tolerance", joint_tolerance_);
        this->get_parameter("left_joint_names", left_joint_names_);
        this->get_parameter("right_joint_names", right_joint_names_);

        // 初始化当前关节值存储容器 (大小为 n_dof_)
        current_left_joints_.resize(n_dof_, 0.0);
        current_right_joints_.resize(n_dof_, 0.0);
        joints_received_ = false;

        // 2. 初始化发布者
        left_hand_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("/left_hand_control", 10);
        right_hand_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("/right_hand_control", 10);

        // 3. 初始化订阅者
        left_task_sub_ = this->create_subscription<dualarm_motion_planning::msg::TaskActionList>(
            "/leftCoordTask", 10, std::bind(&TaskDistributionNode::leftTaskCallback, this, std::placeholders::_1));
        
        right_task_sub_ = this->create_subscription<dualarm_motion_planning::msg::TaskActionList>(
            "/rightCoordTask", 10, std::bind(&TaskDistributionNode::rightTaskCallback, this, std::placeholders::_1));

        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&TaskDistributionNode::jointStateCallback, this, std::placeholders::_1));

        // 4. 初始化客户端
        client_ = this->create_client<dualarm_motion_planning::srv::GiveGoal>("give_goal_1_2");
        
        RCLCPP_INFO(this->get_logger(), "任务分发节点已启动，等待 /joint_states...");
    }

    // 主运行逻辑
    void run()
    {
        rclcpp::Rate loop_rate(10);
        
        // 等待关节状态初始化
        while (rclcpp::ok()) 
        {
            {
                std::lock_guard<std::mutex> lock(joint_state_mutex_);
                if (joints_received_) break; // 只要接收到并匹配成功一次即可
            }
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "等待关节状态数据...");
            loop_rate.sleep();
        }

        RCLCPP_INFO(this->get_logger(), "关节状态就绪，等待任务序列...");

        while (rclcpp::ok()) 
        {
            if (left_tasks_received_ && right_tasks_received_)  // 任务序列接收完成
            {
                // 发送双臂任务
                if (task_index_ < std::min(left_task_sequence_size_, right_task_sequence_size_) && 
                    left_task_completed_ && right_task_completed_)
                {
                    rclcpp::sleep_for(std::chrono::milliseconds(500));

                    RCLCPP_INFO(this->get_logger(), "分配任务 %d .", (task_index_ + 1));

                    // 左右臂任务分发
                    dispatchTask(left_arm_tasks_[task_index_], 1);
                    dispatchTask(right_arm_tasks_[task_index_], 2);

                    task_index_++; 
                }

                // 检查任务是否全部分发完成
                if (task_index_ >= std::min(left_task_sequence_size_, right_task_sequence_size_)) 
                {
                    RCLCPP_INFO(this->get_logger(), "所有任务分配完成!");
                    left_tasks_received_ = false;  
                    right_tasks_received_ = false;
                    task_index_ = 0;
                    left_task_sequence_size_ = 0;
                    right_task_sequence_size_ = 0;
                    left_arm_tasks_.clear(); 
                    right_arm_tasks_.clear();
                }
            }
            loop_rate.sleep();
        }
    }

private:
    // 参数
    double v_max_;
    double a_max_;
    int n_dof_;
    double joint_tolerance_;
    // std::string package_path_;

    int left_n_via_ = 2;
    int right_n_via_ = 2;

    // 状态变量
    std::vector<Task> left_arm_tasks_;
    std::vector<Task> right_arm_tasks_;
    bool left_tasks_received_ = false;
    bool right_tasks_received_ = false;
    int task_index_ = 0;
    int left_task_sequence_size_ = 0;
    int right_task_sequence_size_ = 0;

    // 关节名称定义
    std::vector<std::string> left_joint_names_;
    std::vector<std::string> right_joint_names_;
    
    // 有序的当前关节值缓存
    std::vector<double> current_left_joints_;
    std::vector<double> current_right_joints_;
    bool joints_received_;

    // 任务完成标志 (原子操作或加锁)
    std::atomic<bool> left_task_completed_{true};
    std::atomic<bool> right_task_completed_{true};

    // ROS 2 接口
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr left_hand_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr right_hand_pub_;
    rclcpp::Subscription<dualarm_motion_planning::msg::TaskActionList>::SharedPtr left_task_sub_;
    rclcpp::Subscription<dualarm_motion_planning::msg::TaskActionList>::SharedPtr right_task_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Client<dualarm_motion_planning::srv::GiveGoal>::SharedPtr client_;

    // 数据保护
    std::mutex joint_state_mutex_;

    // 回调函数：解析无序的 JointState 消息
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(joint_state_mutex_);
        
        // 遍历消息中的每一个关节
        for (size_t i = 0; i < msg->name.size(); ++i)
        {
            // 检查是否属于左臂
            for (int j = 0; j < n_dof_; ++j) {
                if (msg->name[i] == left_joint_names_[j]) {
                    current_left_joints_[j] = msg->position[i];
                    break;
                }
            }
            // 检查是否属于右臂
            for (int j = 0; j < n_dof_; ++j) {
                if (msg->name[i] == right_joint_names_[j]) {
                    current_right_joints_[j] = msg->position[i];
                    break;
                }
            }
        }
        
        // 这里我们只要收到过一次完整长度的消息就算 Ready
        if (msg->name.size() >= 14) {
             joints_received_ = true;
        }
    }

    void leftTaskCallback(const dualarm_motion_planning::msg::TaskActionList::SharedPtr msg) 
    {
        left_arm_tasks_.clear(); 
        left_task_sequence_size_ = msg->actions.size(); 

        for (const auto& action : msg->actions) 
        {
            Task task;
            task.action_name = action.action_name;
            for (size_t i = 0; i < action.target_position.size(); ++i) {
                task.goal_data.push_back(action.target_position[i]);
            }
            task.v_max = action.velocity;
            task.a_max = action.acceleration;
            task.action_type = static_cast<ActionType>(action.action_type);
            left_arm_tasks_.push_back(task);
        }

        RCLCPP_INFO(this->get_logger(), "接收到左臂任务序列.");
        left_tasks_received_ = true;
    }

    void rightTaskCallback(const dualarm_motion_planning::msg::TaskActionList::SharedPtr msg) 
    {
        right_arm_tasks_.clear();
        right_task_sequence_size_ = msg->actions.size();

        for (const auto& action : msg->actions) 
        {
            Task task;
            task.action_name = action.action_name;
            for (size_t i = 0; i < action.target_position.size(); ++i) {
                task.goal_data.push_back(action.target_position[i]);
            }
            task.v_max = action.velocity;
            task.a_max = action.acceleration;
            task.action_type = static_cast<ActionType>(action.action_type);
            right_arm_tasks_.push_back(task);
        }

        RCLCPP_INFO(this->get_logger(), "接收到右臂任务序列.");
        right_tasks_received_ = true;
    }

    // 监控任务状态线程
    void monitorTaskStatus(int robot_id, const std::vector<double> goal)
    {
        rclcpp::Rate rate(100);
        while (rclcpp::ok()) 
        {
            bool reached = true;
            {
                std::lock_guard<std::mutex> lock(joint_state_mutex_);
                // 使用排序好的数组进行判断
                if (!joints_received_) {
                    reached = false; 
                } else {
                    for (int i = 0; i < n_dof_; ++i)
                    {
                        if (robot_id == 1) {
                            if (std::abs(current_left_joints_[i] - goal[i]) > joint_tolerance_) {
                                reached = false; break;
                            }
                        } else if (robot_id == 2) {
                            if (std::abs(current_right_joints_[i] - goal[i]) > joint_tolerance_) {
                                reached = false; break;
                            }
                        }
                    }
                }
            }

            if (reached) 
            {
                if (robot_id == 1) left_task_completed_ = true;
                else right_task_completed_ = true;

                RCLCPP_INFO(this->get_logger(), "%s 到达关节目标位置.", (robot_id == 1) ? "左臂" : "右臂");
                break;
            }
            rate.sleep();
        }
    }

    void handleJointSpaceTask(const Task& task, int robot_id) 
    {
        auto req = std::make_shared<dualarm_motion_planning::srv::GiveGoal::Request>();
        std::vector<double> joint_target = task.goal_data;

        // 获取当前关节状态（使用排序好的数组）
        std::vector<double> current_joints_sorted;
        {
            std::lock_guard<std::mutex> lock(joint_state_mutex_);
            if (robot_id == 1) current_joints_sorted = current_left_joints_;
            else current_joints_sorted = current_right_joints_;
        }

        if (robot_id == 1)
        {
            req->qgoal.resize(left_n_via_ * n_dof_);
            for (int i = 0; i < n_dof_; ++i) {
                req->qgoal[i] = current_joints_sorted[i]; // 起点
                req->qgoal[i + n_dof_] = joint_target[i]; // 终点
            }
        }
        else if(robot_id == 2)
        {
            req->qgoal.resize(right_n_via_ * n_dof_);
            for (int i = 0; i < n_dof_; ++i) {
                req->qgoal[i] = current_joints_sorted[i]; // 起点
                req->qgoal[i + n_dof_] = joint_target[i]; // 终点
            }
        }

        req->v_max = task.v_max;
        req->a_max = task.a_max;
        req->robotid = robot_id;

        // 异步调用服务
        if (!client_->wait_for_service(std::chrono::seconds(1))) {
             RCLCPP_ERROR(this->get_logger(), "服务未就绪");
             return;
        }

        auto result_future = client_->async_send_request(req);
        
        // 等待服务返回
        if (result_future.wait_for(std::chrono::seconds(2)) == std::future_status::ready)
        {
            auto response = result_future.get();
            if (response->received == 1) 
            {
                RCLCPP_INFO(this->get_logger(), "机械臂 %d 任务已接收.", robot_id);
                
                if (robot_id == 1) left_task_completed_ = false;
                else right_task_completed_ = false;

                // 启动监控线程
                std::thread(&TaskDistributionNode::monitorTaskStatus, this, robot_id, joint_target).detach();
            }
        }
        else
        {
             RCLCPP_ERROR(this->get_logger(), "机械臂 %d 发送任务超时或失败.", robot_id);
        }
    }

    void handleHandActionTask(const Task& task, int robot_id) 
    {
        std_msgs::msg::Int32MultiArray hand_msg;
        for (size_t i = 0; i < task.goal_data.size(); ++i) {
            hand_msg.data.push_back(static_cast<int>(task.goal_data[i]));
        }

        if (robot_id == 1) 
        {
            left_task_completed_ = false;
            left_hand_pub_->publish(hand_msg);
            RCLCPP_INFO(this->get_logger(), "已发送左手控制指令.");
            left_task_completed_ = true;
        } 
        else if (robot_id == 2) 
        {
            right_task_completed_ = false;
            right_hand_pub_->publish(hand_msg);
            RCLCPP_INFO(this->get_logger(), "已发送右手控制指令.");
            right_task_completed_ = true;
        }
    }

    void dispatchTask(const Task& task, int robot_id) 
    {
        switch (task.action_type)
        {
            case HAND_ACTION:
                handleHandActionTask(task, robot_id);
                break;
            case JOINT_SPACE:
                handleJointSpaceTask(task, robot_id);
                break;
            default:
                RCLCPP_WARN(this->get_logger(), "Unknown action type!");
                break;
        }
    }
};

int main(int argc, char** argv) 
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TaskDistributionNode>();

    // 使用多线程执行器：主线程跑 run() 逻辑，executor 线程跑 回调函数
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    
    std::thread spinner_thread([&executor](){
        executor.spin();
    });

    node->run();

    executor.cancel();
    spinner_thread.join();
    rclcpp::shutdown();
    return 0;
}