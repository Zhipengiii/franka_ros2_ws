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

    '''
        无干涉任务目标 : task1 ~ task6
        有干涉任务目标 : task7 ~ task8
    '''

    # 定义左臂任务序列
    left_tasks = [
        TaskAction(
            action_name="left_task1",
            target_position=[0.338149120939511,
                            0.221033483484982,
                            1.11721935597819,
                            -1.71411646572071,
                            2.81767568979157,
                            2.84872097298062,
                            -0.750589930113507,
                            ],
            velocity=1.0, 
            acceleration=5.0,
            action_type=2  # 关节空间任务
        ),
        TaskAction(
            action_name="left_task2",
            target_position=[2.71928620021207,
                            0.708239146900675,
                            -1.3296850902088,
                            -1.49360439413737,
                            2.77678952480008,
                            2.9296296111625,
                            0.175762971589251,
                            ],
            velocity=1.0,
            acceleration=5.0,
            action_type=2  # 关节空间任务
        ),
        TaskAction(
            action_name="left_task3",
            target_position=[-0.10051553464631,
                            -1.15650858610924,
                            1.62840288764457,
                            -1.37734488768404,
                            2.87622997201076,
                            3.24676795115465,
                            1.18364823171226
                            ],
            velocity=1.0,
            acceleration=5.0,
            action_type=2  # 关节空间任务
        ),
        TaskAction(
            action_name="left_task4",
            target_position=[-0.000594964476430734,
                            -1.29237917686046,
                            2.01514070467243,
                            -1.58100640820653,
                            2.82680053921007,
                            3.2263515038439,
                            1.69355035018885,
                            ],
            velocity=1.0,
            acceleration=5.0,
            action_type=2  # 关节空间任务
        ),
        TaskAction(
            action_name="left_task5",
            target_position=[0.0685166266820106,
                            -1.36334815676696,
                            1.88693438455792,
                            -2.05117514652479,
                            2.87623723346852,
                            2.77367329109661,
                            1.70652540341717,
                            ],
            velocity=1.0,
            acceleration=5.0,
            action_type=2  # 关节空间任务
        ),
        TaskAction(
            action_name="left_task6",
            target_position=[-0.311999372110413,
                            -1.2803196989883,
                            2.0798009924691,
                            -2.32971876402072,
                            2.75700193712705,
                            2.27731649531465,
                            1.4751943277599,
                            ],
            velocity=1.0,
            acceleration=5.0,
            action_type=2  # 关节空间任务
        ),
        TaskAction(
            action_name="left_task7",
            target_position=[1.58577260370472,
                            1.64516409556938,
                            -0.114200996071594,
                            -1.34868277221775,
                            0.171720289234984,
                            3.46115662057646,
                            3.02359195033002,
                            ],
            velocity=1.0,
            acceleration=5.0,
            action_type=2  # 关节空间任务
        ),
        TaskAction(
            action_name="left_task8",
            target_position=[1.78074361359936,
                            1.63590165362402,
                            0.0801052706071601,
                            -1.37244032211925,
                            0.268246539803102,
                            3.28536428912487,
                            3.00529081473285,
                            ],
            velocity=1.0,
            acceleration=5.0,
            action_type=2  # 关节空间任务
        )
    ]
    

    # 定义右臂任务序列
    right_tasks = [
        TaskAction(
            action_name="right_task1",
            target_position=[-0.708187247592792,
                            0.193866496411561,
                            -0.774975086327466,
                            -1.5516230139884,
                            -2.54830754784488,
                            2.9579321010099,
                            1.11664233826901,
                            ],
            velocity=1.0,
            acceleration=5.0,
            action_type=2  # 关节空间任务
        ),
        TaskAction(
            action_name="right_task2",
            target_position=[-2.87105909611051,
                            0.600926718633396,
                            1.41558613562007,
                            -1.50781950245833,
                            -2.81273972295589,
                            3.00755678130092,
                            1.25959634689323,
                            ],
            velocity=1.0,
            acceleration=5.0,
            action_type=2  # 关节空间任务
        ),
        TaskAction(
            action_name="right_task3",
            target_position=[-2.83820011245765,
                            1.02585486246171,
                            1.37126271867444,
                            -1.00480557331624,
                            -2.8762326749071,
                            3.43378767922204,
                            0.479277356901238,
                            ],
            velocity=1.0,
            acceleration=5.0,
            action_type=2  # 关节空间任务
        ),
        TaskAction(
            action_name="right_task4",
            target_position=[0.512574264114717,
                            -1.16410651634981,
                            -2.5964355376164,
                            -1.38808934979503,
                            -0.896178503208254,
                            3.94730873228521,
                            -1.29133551272065,
                            ],
            velocity=1.0,
            acceleration=5.0,
            action_type=2  # 关节空间任务
        ),
        TaskAction(
            action_name="right_task5",
            target_position=[0.69852001727994,
                            -1.41802698505314,
                            -2.06551525068493,
                            -2.16408861013859,
                            -0.239960694141441,
                            4.60427089132721,
                            -2.48593711065832,
                            ],
            velocity=1.0,
            acceleration=5.0,
            action_type=2  # 关节空间任务
        ),
        TaskAction(
            action_name="right_task6",
            target_position=[0.655406536881628,
                            -1.17563772271672,
                            -2.42855965947425,
                            -1.9181918697981,
                            0.192018618588893,
                            3.94367501889909,
                            -2.67859693501348,
                            ],
            velocity=1.0,
            acceleration=5.0,
            action_type=2  # 关节空间任务
        ),
        TaskAction(
            action_name="right_task7",
            target_position=[0.920098279549726,
                            -1.54454201253665,
                            -2.8508801811291,
                            -1.28260228523246,
                            -0.32427625529282,
                            2.88361239410001,
                            -2.08882078953012,
                            ],
            velocity=1.0,
            acceleration=5.0,
            action_type=2  # 关节空间任务
        ),
        TaskAction(
            action_name="right_task8",
            target_position=[1.60574748720504,
                            -1.47421499090178,
                            -2.82207053233401,
                            -1.45958257631123,
                            -1.47449017029752,
                            2.9398140901056,
                            -0.257600197323952,
                            ],
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