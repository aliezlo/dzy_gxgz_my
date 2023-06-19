#ifndef REVSURFACE_HPP
#define REVSURFACE_HPP
// 参数曲面
#include <tuple>

#include "bound.hpp"
#include "curve.hpp"
#include "object3d.hpp"
#include "triangle.hpp"
const int resolution = 10;
const int NEWTON_STEPS = 20;
const float NEWTON_EPS = 1e-4;
class RevSurface : public Object3D {
public:    
    Curve *pCurve; // 旋转曲面的截面曲线
    AABB aabb; //包围盒
    
    typedef std::tuple<unsigned, unsigned, unsigned> Tup3u;
    std::vector<Triangle> triangles;
    
public:
    RevSurface(Curve *pCurve, Material *material)
        : pCurve(pCurve), Object3D(material) {
        // 检查曲线是否在xy平面上
        for (const auto &cp : pCurve->getControls()) {
            if (cp.z() != 0.0) {
                printf("Profile of revSurface must be flat on xy plane.\n");
                exit(0);
            }
        }
        // 设置包围盒参数
        aabb.set(Vector3f(-pCurve->radius, pCurve->ymin - 3, -pCurve->radius),
                 Vector3f(pCurve->radius, pCurve->ymax + 3, pCurve->radius));
        
    }

    ~RevSurface() override { delete pCurve; }

    inline bool intersect(const Ray &r, Hit &h) override {
        // return meshIntersect(r, h, tmin);
        return newtonIntersect(r, h);
    }

    bool newtonIntersect(const Ray &r, Hit &h) {
        float t, theta, mu;
        if (!aabb.intersect(r, t) || t > h.getT()) return false;
        getUV(r, t, theta, mu);
        Vector3f normal, point;
        // cout << "begin!" << endl;
        if (!newton(r, t, theta, mu, normal, point)) {
            // cout << "Not Intersect! t:" << t << " theta: " << theta / (2 *
            // M_PI)
            //      << " mu: " << mu << endl;
            return false;
        }
        if (!isnormal(mu) || !isnormal(theta) || !isnormal(t)) return false;
        if (t < 0 || mu < pCurve->range[0] || mu > pCurve->range[1] ||
            t > h.getT())
            return false;
        h.set(t, material, normal.normalized(),
              material->getColor(theta / (2 * M_PI), mu), point);
        // cout << "Intersect! t:" << t << " theta: " << theta / (2 * M_PI)
        //      << " mu: " << mu << endl;
        return true;
    }

    // bool meshIntersect(const Ray &r, Hit &h, float tmin) {
    //     if (triangles.size() == 0) meshInit();
    //     float tb;
    //     if (!aabb.intersect(r, tb)) return false;
    //     if (tb > h.getT()) return false;
    //     bool flag = false;
    //     for (auto triangle : triangles) flag |= triangle.intersect(r, h,
    //     tmin); return flag;
    // }

    bool newton(const Ray &r, float &t, float &theta, float &mu,
                Vector3f &normal, Vector3f &point) {
        Vector3f dmu, dtheta;
        for (int i = 0; i < NEWTON_STEPS; ++i) {
            if (theta < 0.0) theta += 2 * M_PI;
            if (theta >= 2 * M_PI) theta = fmod(theta, 2 * M_PI);
            if (mu >= 1) mu = 1.0 - FLT_EPSILON;
            if (mu <= 0) mu = FLT_EPSILON;
            point = getPoint(theta, mu, dtheta, dmu);
            Vector3f f = r.origin + r.direction * t - point;
            float dist2 = f.squaredLength();
            // cout << "Iter " << i + 1 << " t: " << t
            //      << " theta: " << theta / (2 * M_PI) << " mu: " << mu
            //      << " dist2: " << dist2 << endl;
            normal = Vector3f::cross(dmu, dtheta);
            if (dist2 < NEWTON_EPS) return true;
            float D = Vector3f::dot(r.direction, normal);
            t -= Vector3f::dot(dmu, Vector3f::cross(dtheta, f)) / D;
            mu -= Vector3f::dot(r.direction, Vector3f::cross(dtheta, f)) / D;
            theta += Vector3f::dot(r.direction, Vector3f::cross(dmu, f)) / D;
        }
        return false;
    }

    Vector3f getPoint(const float &theta, const float &mu, Vector3f &dtheta,
                      Vector3f &dmu) {
        Vector3f pt;
        Quat4f rot;
        rot.setAxisAngle(theta, Vector3f::UP);
        Matrix3f rotMat = Matrix3f::rotation(rot);
        CurvePoint cp = pCurve->getPoint(mu);
        pt = rotMat * cp.V;
        dmu = rotMat * cp.T;
        dtheta = Vector3f(-cp.V.x() * sin(theta), 0, -cp.V.x() * cos(theta));
        return pt;
    }

    void getUV(const Ray &r, const float &t, float &theta, float &mu) {
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
