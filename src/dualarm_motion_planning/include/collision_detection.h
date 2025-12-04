#ifndef COLLISION_DETECTION_H
#define COLLISION_DETECTION_H

#include <array>
#include <Eigen/Dense>

// 定义 clamp 函数，因为 C++ 标准库在 C++17 之前没有提供该函数
// 该函数用于限制值在指定范围内
template<typename T>
T clamp(T value, T min, T max) {
    return (value < min) ? min : ((value > max) ? max : value);
}

// 碰撞检测函数声明
bool collision_detection(std::array<double, 3> s1_v1, std::array<double, 3> s1_v2,
                         std::array<double, 3> s2_v1, std::array<double, 3> s2_v2,
                         double collision_distance);

#endif // COLLISION_DETECTION_H    