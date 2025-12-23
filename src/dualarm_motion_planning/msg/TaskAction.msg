# 实验ID
string experiment_id  

# 目标位置，使用可变长度数组存储 6 或 7 个 double 类型的值
float64[] target_position

# 速度约束
float64 velocity

# 加速度
float64 acceleration

# 动作类型，使用 int32 类型
int32 action_type 

# 任务干涉类型 (FREE, COLLISION_NO_BLOCK, BLOCKING)
int32 task_type