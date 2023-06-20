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
        bool sign[3];
        float t_near[3], t_far[3];
        int k;

        // 计算每个坐标轴上射线对应的 t 值范围
        for (int i = 0; i < 3; i++) {
            t_near[i] = (bounds[sign[i] = invdir[i] < 0 ? 1 : 0][i] - o[i]) * invdir[i];
            t_far[i] = (bounds[!sign[i]][i] - o[i]) * invdir[i];
        }

        // 找到 t 值范围的最大值和最小值
        float t_max_near = std::max(std::max(t_near[0], t_near[1]), t_near[2]);
        float t_min_far = std::min(std::min(t_far[0], t_far[1]), t_far[2]);

        // 判断是否有相交
        if (t_max_near <= t_min_far) {
            t_min = t_max_near;
            return true;
        }
        else {
            return false;
        }
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

        // 计算OC在射线方向的投影长度OH: OC点乘OH/|OH|
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
