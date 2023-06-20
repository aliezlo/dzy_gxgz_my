#ifndef REVSURFACE_HPP
#define REVSURFACE_HPP
// 参数曲面
#include <tuple>

#include "bound.hpp"
#include "curve.hpp"
#include "object3d.hpp"
#include "triangle.hpp"

const int resolution = 10; // 曲面离散化时的分辨率
const int NEWTON_STEPS = 20; // 牛顿迭代求解交点最大迭代次数
const float NEWTON_EPS = 1e-4; // 牛顿迭代求解交点的误差阈值
const float NEWTON_DELTA = 1e-6; // 牛顿迭代求解交点的步长

class RevSurface : public Object3D {
private:
    Curve *pCurve; // 绕z轴旋转而来的曲线
    AABB aabb; // 该物体的包围盒
    // 下面三个参数用于表示可绘制表面
    typedef std::tuple<unsigned, unsigned, unsigned> Tup3u;
    // 三角形面片
    std::vector<Triangle> triangles;

public:
    // 构造函数
    RevSurface(Curve *pCurve, Material *material) :
        pCurve(pCurve), Object3D(material)
    {
        // 检查曲线是否在xy平面上
        for (const auto &cp : pCurve->getControls()) {
            if (cp.z() != 0.0) {
                printf("Profile of revSurface must be flat on xy plane.\n");
                exit(0);
            }
        }
        // 计算并设置物体的包围盒
        aabb.set(Vector3f(-pCurve->radius, pCurve->ymin - 3, -pCurve->radius),
                 Vector3f(pCurve->radius, pCurve->ymax + 3, pCurve->radius));
    }

    

    ~RevSurface() override { delete pCurve; }

    inline bool intersect(const Ray &r, Hit &h) override {
        // t：光线与包围盒的交点距离；theta：光线与z轴的夹角；mu：光线与曲线的交点在参数空间内的坐标
        float t, theta, mu;
        // 与包围盒不相交，则不存在交点，返回 false
        if (!aabb.intersect(r, t) || t > h.getT()) return false;
        // 获取光线和曲面的交点在参数空间内的坐标
        getUV(r, t, theta, mu);
        Vector3f normal, point;
        // 如果使用牛顿迭代法无法求解交点，则不存在交点，返回 false
        if (!newton(r, t, theta, mu, normal, point)) {
            return false;
        }
        // 如果交点参数不在有效范围内或者交点距离大于当前最近的交点距离，则不存在交点，返回 false
        if (!isnormal(mu) || !isnormal(theta) || !isnormal(t)) return false;
        if (t < 0 || mu < pCurve->range[0] || mu > pCurve->range[1] || t > h.getT())
            return false;
        // 设置 Hit 结构体记录碰撞信息，返回 true 表示存在交点
        h.set(t, material, normal.normalized(),
            material->getColor(theta / (2 * M_PI), mu), point);
        return true;
    }

    bool newton(const Ray &ray, float &t, float &theta, float &mu,
            Vector3f &normal, Vector3f &point) {
        Vector3f dmu, dtheta;
        // 进行牛顿迭代求解光线和曲面的交点
        for (int i = 0; i < NEWTON_STEPS; ++i) {
            // 控制分段宽度变化
            if (theta < 0.0) theta += 2 * M_PI;
            if (theta >= 2 * M_PI) theta -= 2 * M_PI;
            if (mu > 1.0 - FLT_EPSILON) mu = 1.0 - FLT_EPSILON;
            if (mu < FLT_EPSILON) mu = FLT_EPSILON;
            // 计算当前迭代点在曲面上的坐标、偏导数和法向量
            point = getPoint(theta, mu, dtheta, dmu);
            Vector3f f = ray.origin + ray.direction * t - point;
            float dist2 = f.squaredLength();
            // 如果当前点与曲面距离已经足够小，则认为找到了交点，返回 true
            if (dist2 < NEWTON_EPS) return true;
            // 计算方程需要使用的值
            normal = Vector3f::cross(dmu, dtheta);
            float D = Vector3f::dot(ray.direction, normal);
            // 如果 D 非常接近 0，则已经无法继续迭代，故返回 false 表示无法求解交点。
            if (std::abs(D) <= NEWTON_DELTA) return false;
            // 更新牛顿迭代的参数
            float dt = Vector3f::dot(dmu, Vector3f::cross(dtheta, f)) / D;
            float dmu_ = Vector3f::dot(ray.direction, Vector3f::cross(dtheta, f)) / D;
            float dtheta_ = Vector3f::dot(ray.direction, Vector3f::cross(dmu, f)) / D;
            // 检查当前计算的参数值是否有越界的情况
            if (mu - dmu_ < FLT_EPSILON || mu - dmu_ > 1.0 - FLT_EPSILON) {
                return false;
            }
            theta += dtheta_;
            mu -= dmu_;
            t -= dt;
        }
        // 如果迭代次数达到了上限仍未找到交点，则返回 false 表示不存在交点
        return false;
    }


    Vector3f getPoint(const float &theta, const float &mu, Vector3f &dtheta, Vector3f &dmu) {
        // 首先，根据经度 theta 构建旋转矩阵 rotMat
        Quat4f rot;
        rot.setAxisAngle(theta, Vector3f::UP);
        Matrix3f rotMat = Matrix3f::rotation(rot);

        // 然后，调用 Curve 类的 getPoint 方法，获取纬度 mu 处的点 cp
        CurvePoint cp = pCurve->getPoint(mu);
        // 将 cp 中的点 P 乘上旋转矩阵 rotMat，得到最终的点坐标 pt
        Vector3f pt = rotMat * cp.V;

        // 计算经度方向和纬度方向的偏导数 dtheta 和 dmu
        dmu = rotMat * cp.T;
        dtheta = Vector3f(-cp.V.x() * sin(theta), 0, -cp.V.x() * cos(theta));
        // 最后，返回点 P 的坐标
        return pt;
    }

    void getUV(const Ray &r, const float &t, float &theta, float &mu) {
    // 根据交点 pt 计算其球面坐标系下的经度 theta 和纬度 mu
        Vector3f pt(r.origin + r.direction * t);
        theta = atan2(-pt.z(), pt.x()) + M_PI;
        mu = (pCurve->ymax - pt.y()) / (pCurve->ymax - pCurve->ymin);
    }

    Vector3f min() const override { return aabb.bounds[0]; }
    Vector3f max() const override { return aabb.bounds[1]; }
    Vector3f center() const override {
        return (aabb.bounds[0] + aabb.bounds[1]) / 2;
    }
    vector<Object3D *> getFaces() override { return {(Object3D *)this}; }
};

#endif  // REVSURFACE_HPP
