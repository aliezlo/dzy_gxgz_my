#ifndef BOUND_H
#define BOUND_H

#include <vecmath.h>
#include <vector>
#include "constants.h"
#include "ray.hpp"

using std::vector;

// 表示三维空间中的轴对齐盒子
class AABB {
public:

    Vector3f bounds[2];     // 盒子的最小点和最大点坐标

    // 默认构造函数，将bounds[0]初始化为(正无穷，正无穷，正无穷)，将bounds[1]初始化为(负无穷，负无穷，负无穷)
    AABB() {
        bounds[0] = Vector3f(INF);
        bounds[1] = Vector3f(-INF);
    }

    // 构造函数，将bounds[0]初始化为min，将bounds[1]初始化为max
    AABB(const Vector3f &min, const Vector3f &max) {
        bounds[0] = min;
        bounds[1] = max;
    }

    // 设置盒子的最小点和最大点坐标
    void set(const Vector3f &lo, const Vector3f &hi) {
        bounds[0] = lo;
        bounds[1] = hi;
    }

    // 更新盒子的最小点和最大点坐标，使其能够覆盖新的点vec
    void updateBound(const Vector3f &vec) {
        for (int i = 0; i < 3; ++i) {
            bounds[0][i] = std::min(bounds[0][i], vec[i]);
            bounds[1][i] = std::max(bounds[1][i], vec[i]);
        }
    }

    // 射线与盒子的相交测试，返回是否相交并传出最小相交距离t_min
    bool intersect(const Ray &r, float &t_min) {
        Vector3f o(r.getOrigin()), invdir(1 / r.getDirection());
        vector<int> sgn = {invdir.x() < 0, invdir.y() < 0, invdir.z() < 0};
        t_min = INF;
        float tmin_x, tmax_x, tmin_y, tmax_y, tmin_z, tmax_z;
        tmin_x = (bounds[sgn[0]].x() - o.x()) * invdir.x();
        tmax_x = (bounds[1 - sgn[0]].x() - o.x()) * invdir.x();
        tmin_y = (bounds[sgn[1]].y() - o.y()) * invdir.y();
        tmax_y = (bounds[1 - sgn[1]].y() - o.y()) * invdir.y();

        // 判断x轴方向与y轴方向的相交情况
        if ((tmin_x > tmax_y) || (tmin_y > tmax_x)) {
            return false;
        }

        // 更新最小和最大相交距离
        tmin_x = std::max(tmin_x, tmin_y);
        tmax_x = std::min(tmax_x, tmax_y);

        tmin_z = (bounds[sgn[2]].z() - o.z()) * invdir.z();
        tmax_z = (bounds[1 - sgn[2]].z() - o.z()) * invdir.z();

        // 判断z轴方向的相交情况
        if ((tmin_x > tmax_z) || (tmin_z > tmax_x)) {
            return false;
        }

        // 更新最小和最大相交距离
        tmin_x = std::max(tmin_x, tmin_z);
        tmax_x = std::min(tmax_x, tmax_z);

        // 将最小相交距离传出
        t_min = tmin_x;

        return true;
    }
};

// 表示三维空间中的圆柱体
class Cylinder {
public:

    float ymin, ymax, radius;   // 圆柱底面y坐标、顶面y坐标和半径

    // 构造函数，设置圆柱底面y坐标、顶面y坐标和半径
    Cylinder(float ymin, float ymax, float r): ymin(ymin), ymax(ymax), radius(r) {}

    // 射线与圆柱的相交测试，返回是否相交并传出相交距离t
    bool intersect(const Ray &r, float &t) {
        Vector3f o(r.getOrigin()), dir(r.getDirection());
        Vector3f OC(-o);

        // 计算OC在射线方向的投影长度OH: OC@OH/|OH|
        float OH = Vector3f::dot(OC, dir);

        // 计算CH的长度
        float CH_2 = OC.squaredLength() - OH * OH;

        // 如果射线与圆柱的距离平方大于半径平方，则不相交
        if (CH_2 > radius * radius) {
            return false;
        }

        // 计算PH的长度
        float PH_2 = radius * radius - CH_2;

        // 圆内相交：圆外相交
        t = (OH <= sqrt(PH_2)) ? OH + sqrt(PH_2) : OH - sqrt(PH_2);

        return true;
    }
};

#endif  // !BOUND_H
