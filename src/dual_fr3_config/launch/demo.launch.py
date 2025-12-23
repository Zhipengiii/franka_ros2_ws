# from moveit_configs_utils import MoveItConfigsBuilder
# from moveit_configs_utils.launches import generate_demo_launch


# def generate_launch_description():
#     moveit_config = MoveItConfigsBuilder("dual_fr3_robot", package_name="dual_fr3_config").to_moveit_configs()
#     return generate_demo_launch(moveit_config)



import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("dual_fr3_robot", package_name="dual_fr3_config")
        .robot_description(file_path="config/dual_fr3_robot.urdf.xacro")
        .robot_description_semantic(file_path="config/dual_fr3_robot.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        # 【关键】确保 demo 环境下也加载优化后的 OMPL 参数
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

    # 启动 RViz 并加载 MotionPlanning 插件
    rviz_config_file = os.path.join(
        moveit_config.package_path, "config", "moveit.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
    )

    # 发布静态 TF 和机器人状态
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # 启动仿真控制器
    ros2_controllers_path = os.path.join(
        moveit_config.package_path, "config", "ros2_controllers.yaml"
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="both",
    )

    # 启动核心 move_group
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    return LaunchDescription(
        [
            rviz_node,
            static_tf_node,
            robot_state_publisher,
            ros2_control_node,
            move_group_node,
        ]
    )