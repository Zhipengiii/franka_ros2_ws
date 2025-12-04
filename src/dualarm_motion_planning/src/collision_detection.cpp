#include "collision_detection.h"

/*
    函数输入 ： 
        s1_v1, s1_v2 : 第一个线段的两个端点坐标(x, y, z)
        s2_v1, s2_v2 : 第二个线段的两个端点坐标
        collision_distance : 碰撞检测距离（两臂杆圆柱横截面半径之和 r1 + r2）
    函数输出 ：
        返回值 : true 表示发生碰撞，false 表示未发生碰撞
*/
bool collision_detection(std::array<double, 3> s1_v1, std::array<double, 3> s1_v2,
                         std::array<double, 3> s2_v1, std::array<double, 3> s2_v2,
                         double collision_distance) 
{
    const double epsilon = 1e-9;
    const double cd_sq = collision_distance * collision_distance;

    // ================== 1. 修正类型转换 ==================
    // 包围盒初始化（正确转换std::array到Eigen）
    Eigen::Vector3d s1_min(s1_v1[0], s1_v1[1], s1_v1[2]);
    Eigen::Vector3d s1_max(s1_v2[0], s1_v2[1], s1_v2[2]);
    Eigen::Vector3d s2_min(s2_v1[0], s2_v1[1], s2_v1[2]);
    Eigen::Vector3d s2_max(s2_v2[0], s2_v2[1], s2_v2[2]);

    // ================ 2. AABB包围盒处理 ==================
    // 包围盒扩展（带碰撞距离）
    for(int i = 0; i < 3; ++i)
    {
        // 线段1包围盒
        if(s1_min[i] > s1_max[i]) std::swap(s1_min[i], s1_max[i]);
        s1_min[i] -= collision_distance;
        s1_max[i] += collision_distance;
        
        // 线段2包围盒
        if(s2_min[i] > s2_max[i]) std::swap(s2_min[i], s2_max[i]);
        s2_min[i] -= collision_distance;
        s2_max[i] += collision_distance;
    }

    // 包围盒重叠检查(判断是否有任意一个维度不重叠)
    if((s1_min[0] > s2_max[0]) || (s2_min[0] > s1_max[0]) ||
       (s1_min[1] > s2_max[1]) || (s2_min[1] > s1_max[1]) ||
       (s1_min[2] > s2_max[2]) || (s2_min[2] > s1_max[2])) 
    {
        return false;
    }

    // =========== 3. 精确距离计算 ：求两线段上点的最小距离 ===========
    Eigen::Vector3d p1(s1_v1[0], s1_v1[1], s1_v1[2]);
    Eigen::Vector3d p2(s1_v2[0], s1_v2[1], s1_v2[2]);
    Eigen::Vector3d q1(s2_v1[0], s2_v1[1], s2_v1[2]);
    Eigen::Vector3d q2(s2_v2[0], s2_v2[1], s2_v2[2]);

    Eigen::Vector3d dir1 = p2 - p1;
    Eigen::Vector3d dir2 = q2 - q1;
    Eigen::Vector3d w0 = p1 - q1;

    double a = dir1.dot(dir1);
    double b = dir1.dot(dir2);
    double c = dir2.dot(dir2);
    double d = dir1.dot(w0);
    double e = dir2.dot(w0);
    double denom = a*c - b*b; // ac-b^2 = dir1^2 * dir2^2 * sin^2(theta)

    double s = 0.0, t = 0.0; // 参数，取值范围[0, 1]

    // 两线段平行
    if(denom < epsilon) 
    {
        s = clamp(-e/c, 0.0, 1.0);
        t = clamp((b*s - d)/a, 0.0, 1.0);
    } 
    else // 克莱姆法则求解 t 和 s，有唯一解
    {
        s = clamp((b*e - c*d)/denom, 0.0, 1.0);
        t = clamp((a*e - b*d)/denom, 0.0, 1.0);
    }

    // 计算两线段上最接近的点,并计算距离
    Eigen::Vector3d closest_p = p1 + t * dir1;
    Eigen::Vector3d closest_q = q1 + s * dir2;
    double dist_sq = (closest_p - closest_q).squaredNorm();

    // ========== 4. 端点投影检查(补充边界情况) ===========
    auto project_dist_sq = [](const Eigen::Vector3d& p,
                              const Eigen::Vector3d& a,
                              const Eigen::Vector3d& b)
    {
        Eigen::Vector3d ap = p - a;
        Eigen::Vector3d ab = b - a;
        double t_proj = ap.dot(ab) / ab.squaredNorm();
        t_proj = clamp(t_proj, 0.0, 1.0); 
        return (a + t_proj*ab - p).squaredNorm();
    };

    // 计算所有可能的投影距离，并取最小值
    dist_sq = std::min({dist_sq,
                       project_dist_sq(p1, q1, q2),
                       project_dist_sq(p2, q1, q2),
                       project_dist_sq(q1, p1, p2),
                       project_dist_sq(q2, p1, p2)});

    // ================== 5. 碰撞判断 ==================               
    return dist_sq <= cd_sq;
}    