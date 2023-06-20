#ifndef MATERIAL_H
#define MATERIAL_H
#include <vecmath.h>

#include <cassert>
#include <iostream>

#include "hit.hpp"
#include "ray.hpp"
#include "texture.h"
using std::cout;
using std::endl;
class Material {
   public:
    explicit Material(const Vector3f &color,
                      const Vector3f &s_color = Vector3f::ZERO, float s = 0,
                      const Vector3f &e_color = Vector3f::ZERO, float r = 1,
                      Vector3f t = Vector3f(1, 0, 0),
                      const char *textureFile = "", const char *bumpFile = "")
        : color(color),
          specularColor(s_color),
          shininess(s),
          emission(e_color),
          refr(r),
          type(t),
          texture(textureFile),
          bump(bumpFile) {}

    virtual ~Material() = default;

    //获取材质的颜色（包括纹理）
    Vector3f getColor(float u, float v) const {
        if (!texture.pic)
            return color;
        else
            return texture.getColor(u, v);
    }

    Vector3f phongShade(const Ray &ray, const Hit &hit,
                        const Vector3f &dirToLight,
                        const Vector3f &lightColor) {
        //phong模型，与原版一致。
        /*原版：Vector3f shaded = Vector3f::ZERO;
        Vector3f L = dirToLight.normalized();
        Vector3f N = hit.getNormal().normalized();
        Vector3f V = -ray.getDirection().normalized();
        Vector3f C = lightColor;
        Vector3f R = 2 * Vector3f::dot(L, N) * N - L;
        Vector3f D = diffuseColor;
        Vector3f S = specularColor; 
        float s = shininess;
        if (Vector3f::dot(L, N) > 0) {
            shaded += Vector3f::dot(L, N) * D * C;
            if (Vector3f::dot(R, V) > 0) {
                shaded += pow(Vector3f::dot(R, V), s) * S * C;
            }
        }
        return shaded;*/
        Vector3f N = hit.getNormal(), V = -ray.getDirection().normalized();
        Vector3f Lx = dirToLight.normalized();
        Vector3f Rx = (2 * (Vector3f::dot(Lx, N)) * N - Lx).normalized();
        Vector3f shaded =
            lightColor *
            (hit.color * relu(Vector3f::dot(Lx, N)) +
             specularColor * (pow(relu(Vector3f::dot(V, Rx)), shininess)));
        return shaded;
    }

    Vector3f color;          // 颜色
    Vector3f specularColor;  // 镜面反射系数
    Vector3f emission;       // 发光系数
    float shininess;         // 高光指数
    float refr;              // 折射率
    Vector3f type;           // 种类
    Texture texture, bump;   // 纹理
    float relu(float x) { return std::max((float)0, x); }
};

#endif  // MATERIAL_H
