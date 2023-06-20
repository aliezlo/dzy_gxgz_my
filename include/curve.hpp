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

// 曲线类
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

    // 曲线离散化方法，将曲线分为 resolution 个点，将每个点和其切线储存在 vector<CurvePoint> data 中
    void discretize(int resolution, std::vector<CurvePoint> &data) {
        resolution *= n / k; // resolution 表示所求的点数，基于阶数 k 进行调整
        data.resize(resolution);// 根据 resolution 对 data 进行初始化
        for (int i = 0; i < resolution; ++i) {// 依次计算每个点的参数值 mu，返回对应的 CurvePoint
            float mu =
                ((float)i / resolution) * (range[1] - range[0]) + range[0];
            data[i] = getPoint(mu);
        }
    }

   protected:
    // 扩展参数，将曲线的参数 t 扩展至 tpad，方便计算
    void pad() {
        int tSize = t.size(); // t 数组的长度
        tpad.resize(tSize + k); // 对 tpad 进行初始化，长度为 t 的长度再加上阶数 k
        for (int i = 0; i < tSize; ++i) tpad[i] = t[i];// 将 t 的值赋给 tpad 的前 tSize 个元素
        for (int i = 0; i < k; ++i) tpad[i + tSize] = t.back(); // 将 t 的最后一个元素赋给 tpad 的后面 k 个位置
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
        n = controls.size();// 控制点个数
        k = n - 1;// 阶数，即控制点个数减一
        range[0] = 0;// 参数范围，0-1
        range[1] = 1;
        t.resize(2 * n);// 参数数组，长度为 2n，前 n 个为 0，后 n 个为 1
        for (int i = 0; i < n; ++i) {
            t[i] = 0;
            t[i + n] = 1;
        }
        pad();// 扩展参数
    }

    // 贝塞尔曲线上的点和切线生成方法，最终生成一条贝塞尔曲线
    CurvePoint getPoint(float mu) override {
        CurvePoint pt;
        // bpos 表示参数 mu 所在的区间，即 mu 属于 [t[bpos], t[bpos+1])
        int bpos = std::upper_bound(t.begin(), t.end(), mu) - t.begin() - 1;
        // 计算参数 mu 所在的区间的控制点
        std::vector<float> s(k + 2, 0), ds(k + 1, 1);
        s[k] = 1;
        // 进行Bernstein多项式的计算
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
        // lsk 表示参数 mu 所在的区间的左侧控制点个数，rsk 表示右侧控制点个数
        int lsk = k - bpos, rsk = bpos + 1 - n;
        // 如果左侧控制点个数大于 0，将左侧控制点移动到 s 的前面，同时将 ds 也移动到前面
        if (lsk > 0) {
            for (int i = lsk; i < s.size(); ++i) {
                s[i - lsk] = s[i];
                ds[i - lsk] = ds[i];
            }
            s.resize(s.size() - lsk);
            ds.resize(ds.size() - lsk);
            lsk = 0;
        }
        // 如果右侧控制点个数大于 0，将右侧控制点移动到 s 的后面，同时将 ds 也移动到后面
        if (rsk > 0) {
            if (rsk < s.size()) {
                s.resize(s.size() - rsk);
                ds.resize(ds.size() - rsk);
            } else {
                s.clear();
                ds.clear();
            }
        }
        // 计算贝塞尔曲线上的点和切线，将其返回
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
        n = controls.size();// 控制点个数
        k = 3; //阶数，固定为 3
        t.resize(n + k + 1); // 参数数组，长度为 n+k+1
        for (int i = 0; i < n + k + 1; ++i) t[i] = (float)i / (n + k); //参数数组的值，均匀分布在 0-1 之间
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
        // B 样条曲线的计算,与Bezier不同，B样条只需要考虑当前区间的控制点即可，具体为当前区间的 k+1 个控制点
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
