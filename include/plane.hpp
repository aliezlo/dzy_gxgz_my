#ifndef PLANE_H
#define PLANE_H

#include <vecmath.h>
#include <cmath>
#include "object3d.hpp"

// TODO: 实现表示无限平面的类Plane
// 平面方程：ax+by+cz=d

class Plane : public Object3D {
   public:
    // 构造函数
    Plane(const Vector3f &normal, float d, Material *m)
        : Object3D(m), normal(normal.normalized()), d(d) {
        // 计算u轴对于平面法向量normal的叉积
        uaxis = Vector3f::cross(Vector3f::UP, normal);
    }

    ~Plane() override = default;

    // 判断射线是否与平面相交，并记录相交信息
    bool intersect(const Ray &r, Hit &h) override {
        Vector3f o(r.getOrigin()), dir(r.getDirection());

        // dir.normalize();
        // 计算射线方向向量dir与法向量normal的点积，并判断是否平行
        float cos = Vector3f::dot(normal, dir);
        if (cos > -1e-6) return false; // 如果大于等于0，则没有交点

        // 计算射线与平面的交点坐标t
        float t = (d - Vector3f::dot(normal, o)) / cos;
        if (t < 0 || t > h.getT()) return false;

        // 根据交点坐标计算相应的uv坐标和法向量，并记录相交信息
        float u, v;
        Vector3f p(o + dir * t);
        getUV(u, v, p);
        h.set(t, material, getNormal(u, v), material->getColor(u, v), p);
        return true;
    }

    // 计算在平面上指定点p的uv坐标，u轴与平面法向量垂直，v轴与平面法向量平行
    void getUV(float &u, float &v, const Vector3f &p) {
        v = p.y(); // v坐标为p点的y坐标
        u = Vector3f::dot(p - d * normal, uaxis); // u坐标为p点到平面上一点的向量投影到u轴上的长度
    }

    // 计算在平面上指定点p的法向量
    Vector3f getNormal(float u, float v) {
        Vector2f grad(0);
        // 如果配置了凹凸贴图，则对法向量进行扰动
        float f = material->bump.getDisturb(u, v, grad);
        if (fabs(f) < FLT_EPSILON) return normal;

        // 计算在平面上指定点的u，v方向上的方向向量（各自取单位长度）
        // 并计算法向量
        if (uaxis.squaredLength() < FLT_EPSILON) return normal;
        return Vector3f::cross(uaxis + normal * grad[0],
                               Vector3f::UP + normal * grad[1])
            .normalized();
    }

    // 返回平面的最小（负）顶点
    Vector3f min() const override {
        return -INF * Vector3f(fabs(normal.x()) < 1 - FLT_EPSILON,
                               fabs(normal.y()) < 1 - FLT_EPSILON,
                               fabs(normal.z()) < 1 - FLT_EPSILON) +
               normal * d;
    }

    // 返回平面的最大（正）顶点
    Vector3f max() const override {
        return INF * Vector3f(fabs(normal.x()) < 1 - FLT_EPSILON,
                              fabs(normal.y()) < 1 - FLT_EPSILON,
                              fabs(normal.z()) < 1 - FLT_EPSILON) +
               normal * d;
    }

    // 返回平面的中心点
    Vector3f center() const override { return normal * d; }

    // 返回包含平面的Object3D对象数组
    vector<Object3D *> getFaces() override { return {(Object3D *)this}; }

   protected:
    Vector3f normal, uaxis;
    float d;
};

#endif  // PLANE_H
