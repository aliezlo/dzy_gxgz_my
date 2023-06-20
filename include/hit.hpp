#ifndef HIT_H
#define HIT_H

#include <vecmath.h>

#include "constants.h"
#include "ray.hpp"
class Material;

//交点类，相比于原来的hit类，增加了一些属性：衰减，法向量，辐射通量，光通量，颜色
class Hit {
   public:
    // constructors
    Hit() {
        material = nullptr;//材质
        t = INF;//交点距离
        r2 = INIT_RADIUS;//距离平方
        attenuation = Vector3f(1);//衰减
        normal = fluxLight = flux = color = Vector3f::ZERO;//法向量，辐射通量，光通量，颜色
        n = 0;
    }

    Hit(float _t, Material *m, const Vector3f &norm) {
        t = _t;
        material = m;
        normal = norm;
        r2 = INIT_RADIUS;
        attenuation = Vector3f(1);
        fluxLight = flux = color = Vector3f::ZERO;
        n = 0;
    }

    float getT() const { return t; }

    Material *getMaterial() const { return material; }

    const Vector3f &getNormal() const { return normal; }
    //重新设置交点，用于sppm中的多次迭代
    void reset(const Vector3f &_d) {
        t = INF;
    }
    void set(float _t, Material *m, const Vector3f &n, const Vector3f &c,
             const Vector3f &_p) {
        t = _t;
        material = m;
        normal = n;
        color = c;
        p = _p;
    }

    float t, r2;
    Material *material;
    Vector3f normal, color, flux, fluxLight, attenuation;
    Vector3f dir, p;
    int n;
};

inline std::ostream &operator<<(std::ostream &os, const Hit &h) {
    os << "Hit <" << h.getT() << ", " << h.getNormal() << ">";
    return os;
}
#endif  // HIT_H
