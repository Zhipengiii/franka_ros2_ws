import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 获取包路径
    config_pkg_path = get_package_share_directory('dual_fr3_config') # 配置文件包
    description_pkg_path = get_package_share_directory('dual_fr3_description') # 描述文件包

    # 2. 读取 URDF 和 SRDF
    urdf_file = os.path.join(description_pkg_path, 'urdf', 'dual_fr3_robot.urdf') # 描述文件路径
    with open(urdf_file, 'r') as inf:
        robot_description_content = inf.read()

    srdf_file = os.path.join(config_pkg_path, 'config', 'dual_fr3_robot.srdf') # 配置文件路径
    try:
        with open(srdf_file, 'r') as inf:
            robot_description_semantic_content = inf.read()
    except FileNotFoundError:
        robot_description_semantic_content = ""
        print("Warning: SRDF not found")

    # 读取 kinematics.yaml 并解析为字典
    kinematics_yaml_path = os.path.join(config_pkg_path, 'config', 'kinematics.yaml')
    with open(kinematics_yaml_path, 'r') as file:
        kinematics_parameters = yaml.safe_load(file)

    # 3. 声明参数
    declared_arguments = [
        DeclareLaunchArgument('sample_time', default_value='0.001'),
        DeclareLaunchArgument('collision_detection_sample_time', default_value='0.05'),
        DeclareLaunchArgument('n_dof', default_value='7'),
        DeclareLaunchArgument('offline_replanning_mode', default_value='1'),
        DeclareLaunchArgument('collision_threshold', default_value='0.06'),
        DeclareLaunchArgument('v_max', default_value='0.5'),
        DeclareLaunchArgument('a_max', default_value='2.0'),
        DeclareLaunchArgument('joint_tolerance', default_value='0.001'),
    ]

    # 参数引用
    sample_time = LaunchConfiguration('sample_time')
    n_dof = LaunchConfiguration('n_dof')
    joint_tolerance = LaunchConfiguration('joint_tolerance')

    # 规划组名称
    r1_group_name = 'left_arm'
    r2_group_name = 'right_arm'

    # 规划组关节名列表
    r1_joint_names_list = [
        'left_fr3_joint1','left_fr3_joint2','left_fr3_joint3','left_fr3_joint4','left_fr3_joint5','left_fr3_joint6','left_fr3_joint7'
    ]
    r2_joint_names_list = [
        'right_fr3_joint1','right_fr3_joint2','right_fr3_joint3','right_fr3_joint4','right_fr3_joint5','right_fr3_joint6','right_fr3_joint7'
    ]

    # 运动链连杆名称列表
    r1_arm_links = [
        'left_fr3_link0', 'left_fr3_link1', 'left_fr3_link2', 'left_fr3_link3', 'left_fr3_link4', 'left_fr3_link5', 'left_fr3_link6', 'left_fr3_link7', 'left_fr3_hand'
    ]
    r2_arm_links = [
        'right_fr3_link0', 'right_fr3_link1', 'right_fr3_link2', 'right_fr3_link3', 'right_fr3_link4', 'right_fr3_link5', 'right_fr3_link6', 'right_fr3_link7', 'right_fr3_hand'
    ]

    # 机器人轨迹控制动作名称
    action_name_r1 = '/left_arm_controller/follow_joint_trajectory'
    action_name_r2 = '/right_arm_controller/follow_joint_trajectory'

    # 4. 定义节点
    # 控制节点 R1
    control_r1 = Node(
        package='dualarm_motion_planning',
        executable='control',
        name='control_r1',
        output='screen',
        parameters=[{
            'robot_id': 1,
            'sample_time': sample_time,
            'n_dof': n_dof,
            'joint_tolerance': joint_tolerance,
            'joint_names': r1_joint_names_list, # 规划组关节名
            'action_name': action_name_r1 # 机器人接收运动轨迹的 action 名称
        }]
    )

    # 控制节点 R2
    control_r2 = Node(
        package='dualarm_motion_planning',
        executable='control',
        name='control_r2',
        output='screen',
        parameters=[{
            'robot_id': 2,
            'sample_time': sample_time,
            'n_dof': n_dof,
            'joint_tolerance': joint_tolerance,
            'joint_names': r2_joint_names_list,
            'action_name': action_name_r2
        }]
    )

    # 协调节点
    coord_node = Node(
        package='dualarm_motion_planning',
        executable='coord_node',
        name='coord_node',
        output='screen',
        parameters=[
            kinematics_parameters, 
            {
                'robot_description': robot_description_content,
                'robot_description_semantic': robot_description_semantic_content,
                'offline_replanning_mode': LaunchConfiguration('offline_replanning_mode'),
                'collision_threshold': LaunchConfiguration('collision_threshold'),
                'collision_detection_sample_time': LaunchConfiguration('collision_detection_sample_time'),
                'n_dof': n_dof,
                'robot_id_pair': [1, 2],
                'r1_group_name': r1_group_name, 
                'r2_group_name': r2_group_name,
                'r1_arm_links': r1_arm_links, # 运动链连杆
                'r2_arm_links': r2_arm_links
            }
        ]
    )

    # 任务分发节点
    task_distribution_node = Node(
        package='dualarm_motion_planning',
        executable='task_distribution',
        name='task_distribution_node',
        output='screen',
        parameters=[{
            'v_max': LaunchConfiguration('v_max'),
            'a_max': LaunchConfiguration('a_max'),
            'n_dof': n_dof,
            'joint_tolerance': joint_tolerance,
            'left_joint_names': r1_joint_names_list,
            'right_joint_names': r2_joint_names_list
        }]
    )

    return LaunchDescription(declared_arguments + [
        control_r1,
        control_r2,
        coord_node,
        task_distribution_node
    ])