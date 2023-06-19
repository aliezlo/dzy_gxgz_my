#ifndef TRIANGLE_H
#define TRIANGLE_H

#include <vecmath.h>
#include <cfloat>
#include <cmath>
#include <iostream>

#include "object3d.hpp"
#include "utils.hpp"

using namespace std;

// 定义 Triangle 类，继承 Object3D 类
class Triangle : public Object3D {
public:
    // 删除默认构造函数，需要传入三个顶点坐标和材质
    Triangle() = delete;
    Triangle(const Vector3f& a, const Vector3f& b, const Vector3f& c, Material* m) :
        // 初始化基类 Object3D，并初始化成员变量（包括三角形的顶点坐标和法向量，以及边界信息等）
        Object3D(m), a(a), b(b), c(c), an(Vector3f::ZERO), bn(Vector3f::ZERO), 
        cn(Vector3f::ZERO) {
        normal = Vector3f::cross((b - a), (c - a)).normalized();
        d = Vector3f::dot(normal, a);
        bound[0] = minE(minE(a, b), c);
        bound[1] = maxE(maxE(a, b), c);
        cen = (a + b + c) / 3;
        nSet = false;
        tSet = false;
    }

    // 判断光线是否与三角形相交，如果相交则计算相关信息，保存在 Hit 结构体中并返回 true
    bool intersect(const Ray& r, Hit& h) override {
        Vector3f o(r.getOrigin()), dir(r.getDirection());
        Vector3f v0v1 = b - a;
        Vector3f v0v2 = c - a;
        Vector3f pvec = Vector3f::cross(dir, v0v2);
        float det = Vector3f::dot(v0v1, pvec);
        if (fabs(det) < FLT_EPSILON) return false;
        float invDet = 1 / det;
        Vector3f tvec = o - a;
        float u = Vector3f::dot(tvec, pvec) * invDet;
        if (u < 0 || u > 1) return false;
        Vector3f qvec = Vector3f::cross(tvec, v0v1);
        float v = Vector3f::dot(dir, qvec) * invDet;
        if (v < 0 || u + v > 1) return false;
        float t = Vector3f::dot(v0v2, qvec) * invDet;
        if (t <= 0 || t > h.getT()) return false;
        Vector3f p(o + dir * t);
        getUV(p, u, v);
        h.set(t, material, getNorm(p), material->getColor(u, v), p);
        return true;
    }

    // 设置三个顶点的法向量
    void setVNorm(const Vector3f& anorm, const Vector3f& bnorm, const Vector3f& cnorm) {
        an = anorm;
        bn = bnorm;
        cn = cnorm;
        nSet = true;
    }

    // 设置三个顶点的 UV 坐标
    void setVT(const Vector2f& _at, const Vector2f& _bt, const Vector2f& _ct) {
        at = _at;
        bt = _bt;
        ct = _ct;
        tSet = true;
    }

    // 获取三角形的边界信息
    Vector3f min() const override { return bound[0]; }
    Vector3f max() const override { return bound[1]; }
    Vector3f center() const override { return cen; }

    // 返回仅包含该三角形的 Object3D 数组
    vector<Object3D*> getFaces() override { return {(Object3D*)this}; }

    // 成员变量：法向量、三角形的三个顶点坐标和边界等信息
    Vector3f normal;
    Vector3f a, b, c, cen;
    Vector2f at, bt, ct;
    Vector3f an, bn, cn;
    Vector3f bound[2];
    float d;
    bool nSet = false;
    bool tSet = false;

protected:
    // 判断某个点是否在三角形内部
    bool inTriangle(const Vector3f& p) {
        float va = Vector3f::dot(Vector3f::cross((b - p), (c - p)), normal),
            vb = Vector3f::dot(Vector3f::cross((c - p), (a - p)), normal),
            vc = Vector3f::dot(Vector3f::cross((a - p), (b - p)), normal);
        return (va >= 0 && vb >= 0 && vc >= 0);
    }

    // 获取某个点处的法向量（根据设置的三个顶点法向量进行插值）
    Vector3f getNorm(const Vector3f& p) {
        if (!nSet) return normal;
        Vector3f va = (a - p), vb = (b - p), vc = (c - p);
        float ra = Vector3f::cross(vb, vc).length(),
            rb = Vector3f::cross(vc, va).length(),
            rc = Vector3f::cross(va, vb).length();
        return (ra * an + rb * bn + rc * cn).normalized();
    }

    // 获取某个点处的 UV 坐标（根据设置的三个顶点 UV 坐标进行插值）
    void getUV(const Vector3f& p, float& u, float& v) {
        if (!tSet) return;
        Vector3f va = (a - p), vb = (b - p), vc = (c - p);
        float ra = Vector3f::cross(vb, vc).length(),
            rb = Vector3f::cross(vc, va).length(),
            rc = Vector3f::cross(va, vb).length();
        Vector2f uv = (ra * at + rb * bt + rc * ct) / (ra + rb + rc);
        u = uv.x();
        v = uv.y();
    }

    // 在三角形内部随机生成一条射线，用于 Monte Carlo 等技术
    Ray randomRay(int axis= -1, long long int seed=0) const override {
        float r1 = random(axis, seed), r2 = random(axis, seed);
        if (r1 + r2 > 1) {
            r1 = 1 - r1;
            r2 = 1 - r2;
        }
        return Ray(r1 * b + r2 * c + (1 - r1 - r2) * a, diffDir(normal, axis, seed));
    }
};

#endif  // TRIANGLE_H
