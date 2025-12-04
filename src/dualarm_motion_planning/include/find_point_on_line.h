#ifndef POINT_MATH_H
#define POINT_MATH_H

#include <stdexcept>
#include <cmath>

// 定义三维点结构体
struct Point {
    double x;
    double y;
    double z;
};

// 计算两点之间的距离（声明）
double distance(const Point& p1, const Point& p2);

// 在直线p1-p2的延长线上找到点p3，p3 = p2 + k*(p2-p1)
Point find_point_on_line(const Point& p1, const Point& p2, double k);

#endif // POINT_MATH_H