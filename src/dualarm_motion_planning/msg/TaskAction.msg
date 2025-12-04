# 任务动作名称
string action_name

# 目标位置，使用可变长度数组存储 6 或 7 个 double 类型的值
float64[] target_position

# 速度约束
float64 velocity

# 加速度
float64 acceleration

# 动作类型，使用 int32 类型
int32 action_type
