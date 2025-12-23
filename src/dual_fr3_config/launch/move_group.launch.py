# from moveit_configs_utils import MoveItConfigsBuilder
# from moveit_configs_utils.launches import generate_move_group_launch


# def generate_launch_description():
#     moveit_config = MoveItConfigsBuilder("dual_fr3_robot", package_name="dual_fr3_config").to_moveit_configs()
#     return generate_move_group_launch(moveit_config)

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 使用 MoveItConfigsBuilder 自动加载配置
    moveit_config = (
        MoveItConfigsBuilder("dual_fr3_robot", package_name="dual_fr3_config")
        .robot_description(file_path="config/dual_fr3_robot.urdf.xacro")
        .robot_description_semantic(file_path="config/dual_fr3_robot.srdf")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        # 【关键】显式声明 OMPL 作为规划后端，MoveIt 会自动寻找 config/ompl_planning.yaml
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

    # 启动 move_group 节点
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
            # 全局增加默认规划超时时间，给 RRT* 等算法更多计算空间
            {"planning_pipelines.ompl.planning_time": 15.0}
        ],
    )

    return LaunchDescription([move_group_node])