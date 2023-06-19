#ifndef SPHERE_H
#define SPHERE_H

#include <vecmath.h>

#include <cmath>
#include "utils.hpp"
#include "object3d.hpp"

class Sphere : public Object3D {
public:
    // 构造函数，初始化球体的圆心、半径以及所属材质
    Sphere(const Vector3f &center, float radius, Material *material)
        : Object3D(material), cen(center), radius(radius) {
    }
    ~Sphere() override = default;

    // 判断光线是否与球体相交，并返回 Hit 结构体对象
    bool intersect(const Ray &r, Hit &h) override {
        Vector3f o(r.getOrigin()), dir(r.getDirection());
        Vector3f OC(cen - o);
        float b = -Vector3f::dot(OC, dir);
        float c = OC.squaredLength() - radius * radius;
        float delta = b * b - c;
        if (delta <= 0) return false;   // delta小于等于0，无实数根，无交点，返回 false
        float sqrt_delta = sqrtf32(delta);
        float t1 = (-b - sqrt_delta), t2 = (-b + sqrt_delta);   // 求解二次方程的两个实数根
        float t;
        if (t1 <= h.getT() && t1 >= 0)  // 找到距离光源最近的交点
            t = t1;
        else if (t2 <= h.getT() && t2 >= 0)
            t = t2;
        else
            return false;
        Vector3f OP(o + dir * t - cen);
        Vector3f normal = OP.normalized();
        float u = 0.5 + atan2(normal.x(), normal.z()) / (2 * M_PI),
              v = 0.5 - asin(normal.y()) / M_PI;   // 计算球体的纹理坐标
        h.set(t, material, getNormal(normal, OP, u, v),
              material->getColor(u, v), o + dir * t);
        return true;
    }

    // 计算球体的法向量，用于着色
    Vector3f getNormal(const Vector3f &n, const Vector3f &p, float u, float v) {
        Vector2f grad(0);
        float f = material->bump.getDisturb(u, v, grad);   // 计算凹凸纹理产生的影响
        if (fabs(f) < FLT_EPSILON) return n;
        float phi = u * 2 * M_PI, theta = M_PI - v * M_PI;
        Vector3f pu(-p.z(), 0, p.x()),
            pv(p.y() * cos(phi), -radius * sin(theta), p.y() * sin(phi));
        if (pu.squaredLength() < FLT_EPSILON) return n;
        return Vector3f::cross(pu + n * grad[0] / (2 * M_PI),
                               pv + n * grad[1] / M_PI)
            .normalized();
    }

    // 随机生成一条射线，从球体上的一个随机点开始出发
    Ray randomRay(int axis=-1, long long int seed=0) const override {
        float u = 2*random(axis, seed) - 1, v = 2*random(axis, seed) - 1;
        float r2 =u * u + v * v;
        while(r2>=1) {
            ++seed;
            u = 2*random(axis, seed) - 1;
            v = 2*random(axis, seed) - 1;
            r2 = u * u + v * v;
        }
        Vector3f dir(2*u*sqrtf(1-r2), 2*v*sqrt(1-r2),1-2*r2);
        dir.normalize();
        return Ray(cen+radius*dir, dir);
    }

    // 返回球体的最小边界
    Vector3f min() const override { return cen - radius; }
    
    // 返回球体的最大边界
    Vector3f max() const override { return cen + radius; }
    
    // 返回球体的圆心坐标
    Vector3f center() const override { return cen; }
    
    // 返回该球体对象的指针
    vector<Object3D *> getFaces() override { return {(Object3D *)this}; }

protected:
    Vector3f cen;   // 球体的圆心坐标
    float radius;   // 球体的半径
};

#endif
