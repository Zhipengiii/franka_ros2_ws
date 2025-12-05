#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from dualarm_motion_planning.msg import TaskAction, TaskActionList
import time

class TaskSender(Node):
    def __init__(self):
        super().__init__('dual_arm_task_tester')
        # 创建任务序列发布者
        self.left_task_pub = self.create_publisher(TaskActionList, '/leftCoordTask', 10)
        self.right_task_pub = self.create_publisher(TaskActionList, '/rightCoordTask', 10)

    def send_task_sequence(self, pub, actions):
        """发送任务序列到指定话题"""
        task_list = TaskActionList()
        task_list.actions = actions
        pub.publish(task_list)
        self.get_logger().info(f"已发送任务序列到 {pub.topic_name}")

def main(args=None):
    # 初始化ROS节点
    rclpy.init(args=args)
    node = TaskSender()
    
    # 等待连接 (简单延时，或者可以使用 wait_for_subscribers 逻辑)
    time.sleep(1.0)
    
    '''
        定义任务序列
        每个任务包括：
        - action_name: 任务名称
        - target_position: 目标位置（关节空间为关节角度列表（单位：弧度），笛卡尔空间为位姿列表）
        - velocity: 最大速度（单位：弧度/秒）
        - acceleration: 最大加速度（单位：弧度/秒²）
        - action_type: 任务类型（0: 笛卡尔空间, 1: 直线空间, 2: 关节空间）
    '''

    # 定义左臂任务序列
    left_tasks = [
        TaskAction(
            action_name="left_task1",
            target_position=[-1.174803, -0.747422, 2.845125, -1.633357, -2.742452, 2.361245, 1.647983],
            velocity=1.0, 
            acceleration=5.0,
            action_type=2  # 关节空间任务
        ),
        TaskAction(
            action_name="left_task2",
            target_position=[-1.022599, -1.836047, 2.414729, -1.449909, -2.542112, 1.984310, 2.416757],
            velocity=1.0,
            acceleration=5.0,
            action_type=2  # 关节空间任务
        )
    ]
    
    # 定义右臂任务序列
    right_tasks = [
        TaskAction(
            action_name="right_task1",
            target_position=[-2.137347, 1.412162, 1.016419, -1.140118, 0.310286, 3.164228, -2.859364],
            velocity=1.0,
            acceleration=5.0,
            action_type=2  # 关节空间任务
        ),
        TaskAction(
            action_name="right_task2",
            target_position=[-1.777525, 1.637401, 0.001949, -1.079258, 0.509751, 2.899927, -1.525326],
            velocity=1.0,
            acceleration=5.0,
            action_type=2  # 关节空间任务
        )
    ]
    
    # 发送任务序列
    node.send_task_sequence(node.left_task_pub, left_tasks)
    node.send_task_sequence(node.right_task_pub, right_tasks)
    
    # 保持节点运行
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()