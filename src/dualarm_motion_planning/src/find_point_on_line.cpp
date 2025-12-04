#include "find_point_on_line.h"

// 计算两点之间的三维欧几里得距离
double distance(const Point& p1, const Point& p2)
{
    return std::sqrt(std::pow(p2.x - p1.x, 2) +
                     std::pow(p2.y - p1.y, 2) +
                     std::pow(p2.z - p1.z, 2));
}

// 在直线p1-p2的延长线上找到点p3
Point find_point_on_line(const Point& p1, const Point& p2, double k) 
{
    // 计算方向向量
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    double dz = p2.z - p1.z;

    // 处理点重合的情况
    double L = distance(p1, p2);
    if (L == 0.0) 
    {
        throw std::invalid_argument("点1和点2重合,无法确定直线");
    }

    // 计算目标点坐标（延长线方向由k的正负决定）
    Point p3;
    p3.x = p2.x + k * dx;
    p3.y = p2.y + k * dy;
    p3.z = p2.z + k * dz;

    return p3;
}