#ifndef CURVE_HPP
#define CURVE_HPP

#include <vecmath.h>
#include <algorithm>
#include <iostream>
#include <utility>
#include <vector>
#include "constants.h"
#include "object3d.hpp"
#include "constants.h"

using namespace std;

// 曲线上的点
struct CurvePoint {
    Vector3f V;  // 点坐标
    Vector3f T;  // 切线（单位向量）
};

class Curve {
   public:
    std::vector<Vector3f> controls;     // 控制点
    std::vector<float> t;               // 参数(0-1)
    std::vector<float> tpad;            // 扩展的参数
    int n;                              // 控制点个数
    int k;                              // 曲线阶数
    float ymin, ymax, radius;           // 其中 ymin 为 y 轴最小值，ymax 为 y 轴最大值，radius 为 x 和 z 轴的最大值
    float range[2];                     // 参数范围

   public:
    Curve(std::vector<Vector3f> points) : controls(std::move(points)) {
        ymin = INF;
        ymax = -INF;
        radius = 0;
        for (auto pt : controls) {
            ymin = min(pt.y(), ymin);
            ymax = max(pt.y(), ymax);
            radius = max(radius, fabs(pt.x()));
            radius = max(radius, fabs(pt.z()));
        }
    }

    // 获取控制点
    std::vector<Vector3f> &getControls() { return controls; }

    // 根据参数获取曲线上的点和切线
    virtual CurvePoint getPoint(float mu) = 0;

    // 曲线离散化
    void discretize(int resolution, std::vector<CurvePoint> &data) {
        resolution *= n / k;
        data.resize(resolution);
        for (int i = 0; i < resolution; ++i) {
            float mu =
                ((float)i / resolution) * (range[1] - range[0]) + range[0];
            data[i] = getPoint(mu);
        }
    }

   protected:
    // 扩展参数
    void pad() {
        int tSize = t.size();
        tpad.resize(tSize + k);
        for (int i = 0; i < tSize; ++i) tpad[i] = t[i];
        for (int i = 0; i < k; ++i) tpad[i + tSize] = t.back();
    }
};

// 贝塞尔曲线
class BezierCurve : public Curve {
   public:
    BezierCurve(const std::vector<Vector3f> &points) : Curve(points) {
        if (points.size() < 4 || points.size() % 3 != 1) {
            printf("Number of control points of BezierCurve must be 3n+1!\n");
            exit(0);
        }
        n = controls.size();
        k = n - 1;
        range[0] = 0;
        range[1] = 1;
        t.resize(2 * n);
        for (int i = 0; i < n; ++i) {
            t[i] = 0;
            t[i + n] = 1;
        }
        pad();
    }

    // 贝塞尔曲线上的点和切线
    CurvePoint getPoint(float mu) override {
        CurvePoint pt;
        int bpos = std::upper_bound(t.begin(), t.end(), mu) - t.begin() - 1;
        std::vector<float> s(k + 2, 0), ds(k + 1, 1);
        s[k] = 1;
        for (int p = 1; p <= k; ++p) {
            for (int ii = k - p; ii < k + 1; ++ii) {
                int i = ii + bpos - k;
                float w1, dw1, w2, dw2;
                if (tpad[i + p] == tpad[i]) {
                    w1 = mu;
                    dw1 = 1;
                } else {
                    w1 = (mu - tpad[i]) / (tpad[i + p] - tpad[i]);
                    dw1 = 1.0 / (tpad[i + p] - tpad[i]);
                }
                if (tpad[i + p + 1] == tpad[i + 1]) {
                    w2 = 1 - mu;
                    dw2 = -1;
                } else {
                    w2 = (tpad[i + p + 1] - mu) /
                         (tpad[i + p + 1] - tpad[i + 1]);
                    dw2 = -1 / (tpad[i + p + 1] - tpad[i + 1]);
                }
                if (p == k) ds[ii] = (dw1 * s[ii] + dw2 * s[ii + 1]) * p;
                s[ii] = w1 * s[ii] + w2 * s[ii + 1];
            }
        }
        s.pop_back();
        int lsk = k - bpos, rsk = bpos + 1 - n;
        if (lsk > 0) {
            for (int i = lsk; i < s.size(); ++i) {
                s[i - lsk] = s[i];
                ds[i - lsk] = ds[i];
            }
            s.resize(s.size() - lsk);
            ds.resize(ds.size() - lsk);
            lsk = 0;
        }
        if (rsk > 0) {
            if (rsk < s.size()) {
                s.resize(s.size() - rsk);
                ds.resize(ds.size() - rsk);
            } else {
                s.clear();
                ds.clear();
            }
        }
        for (int j = 0; j < s.size(); ++j) {
            pt.V += controls[-lsk + j] * s[j];
            pt.T += controls[-lsk + j] * ds[j];
        }
        return pt;
    }
};

// B 样条曲线
class BsplineCurve : public Curve {
   public:
    BsplineCurve(const std::vector<Vector3f> &points) : Curve(points) {
        if (points.size() < 4) {
            printf(
                "Number of control points of BspineCurve must be more than "
                "4!\n");
            exit(0);
        }
        n = controls.size();
        k = 3;
        t.resize(n + k + 1);
        for (int i = 0; i < n + k + 1; ++i) t[i] = (float)i / (n + k);
        pad();
        range[0] = t[k];
        range[1] = t[n];
    }

    // B 样条曲线上的点和切线
    CurvePoint getPoint(float mu) override {
        CurvePoint pt;
        int bpos = std::upper_bound(t.begin(), t.end(), mu) - t.begin() - 1;
        std::vector<float> s(k + 1, 0), ds(k, 0);
        s[k] = 1;
        for (int p = 1; p <= k; ++p) {
            for (int ii = k - p; ii < k + 1; ++ii) {
                int i = ii + bpos - k;
                float w1 = 0, dw1 = 0, w2 = 0, dw2 = 0;
                if (tpad[i + p] != tpad[i]) {
                    w1 = (mu - tpad[i]) / (tpad[i + p] - tpad[i]);
                    dw1 = 1.0 / (tpad[i + p] - tpad[i]);
                }
                if (tpad[i + p + 1] != tpad[i + 1]) {
                    w2 = (tpad[i + p + 1] - mu) /
                         (tpad[i + p + 1] - tpad[i + 1]);
                    dw2 = -1.0 / (tpad[i + p + 1] - tpad[i + 1]);
                }
                if (p == k) ds[ii] = (dw1 * s[ii] + dw2 * s[ii + 1]) * p;
                s[ii] = w1 * s[ii] + w2 * s[ii + 1];
            }
        }
        for (int j = 0; j <= k; ++j) {
            pt.V += controls[bpos - k + j] * s[j];
            pt.T += controls[bpos - k + j] * ds[j];
        }
        return pt;
    }
};

#endif  // CURVE_HPP
