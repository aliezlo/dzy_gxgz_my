#ifndef HIT_KDTREE_H
#define HIT_KDTREE_H

#include <algorithm>
#include "constants.h"
#include "hit.hpp"

class HitKDTreeNode {
public:
    Hit *hit;               // 节点对应的 Hit
    Vector3f min, max;      // 节点对应的空间范围的 min 和 max
    float maxr2;            // 节点所包含的所有 Hit 中半径最大的一个
    HitKDTreeNode *ls, *rs; // 左右子树
};

inline float sqr(float a) { return a * a; }

class HitKDTree {
public:
    int n;
    Hit **hits;
    HitKDTreeNode *root;

    // 以当前节点对应区间内的 Hits 构建 KD 树
    HitKDTreeNode *build(int l, int r, int d) {
        // 创建一个新的节点
        HitKDTreeNode *p = new HitKDTreeNode;
        // 初始化节点属性
        p->min = Vector3f(INF, INF, INF);
        p->max = -p->min;
        p->maxr2 = 0;
        // 计算节点的 min、max 和 maxr2
        for (int i = l; i <= r; ++i) {
            p->min = minE(p->min, hits[i]->p);
            p->max = maxE(p->max, hits[i]->p);
            p->maxr2 = std::max(p->maxr2, hits[i]->r2);
        }
        // 选取当前区间的中间位置，作为节点对应的 Hit
        int m = l + r >> 1;
        if (d == 0)
            std::nth_element(hits + l, hits + m, hits + r + 1, cmpHitX);
        else if (d == 1)
            std::nth_element(hits + l, hits + m, hits + r + 1, cmpHitY);
        else
            std::nth_element(hits + l, hits + m, hits + r + 1, cmpHitZ);
        p->hit = hits[m];
        // 递归构建左右子树，并设置当前节点的左右子树
        if (l <= m - 1)
            p->ls = build(l, m - 1, (d + 1) % 3);
        else
            p->ls = nullptr;
        if (m + 1 <= r)
            p->rs = build(m + 1, r, (d + 1) % 3);
        else
            p->rs = nullptr;
        return p;
    }

    // 递归删除 KD 树中的所有节点
    void del(HitKDTreeNode *p) {
        if (p->ls) del(p->ls);
        if (p->rs) del(p->rs);
        delete p;
    }

public:
    HitKDTree(vector<Hit *> *hits) {
        n = hits->size();
        this->hits = new Hit *[n];
        for (int i = 0; i < n; ++i) this->hits[i] = (*hits)[i];
        // 构建 KD 树
        root = build(0, n - 1, 0);
    }

    ~HitKDTree() {
        if (!root) return;
        // 删除 KD 树中所有节点以及 hits 数组
        del(root);
        delete[] hits;
    }

    // 更新 KD 树中距离 photon 最近的 Hit 的光通量信息
    void update(HitKDTreeNode *p, const Vector3f &photon,
                const Vector3f &attenuation, const Vector3f &d) {
        if (!p) return;
        float mind = 0, maxd = 0;
        // 判断当前点到当前节点所对应空间的最近距离是否超过当前节点所包含的最大半径
        if (photon.x() > p->max.x()) mind += sqr(photon.x() - p->max.x());
        if (photon.x() < p->min.x()) mind += sqr(p->min.x() - photon.x());
        if (photon.y() > p->max.y()) mind += sqr(photon.y() - p->max.y());
        if (photon.y() < p->min.y()) mind += sqr(p->min.y() - photon.y());
        if (photon.z() > p->max.z()) mind += sqr(photon.z() - p->max.z());
        if (photon.z() < p->min.z()) mind += sqr(p->min.z() - photon.z());
        if (mind > p->maxr2) return;
        // 当前点到当前节点所对应的 Hit 的距离小于等于该 Hit 的半径，说明该 Hit 受到了光子的照射
        if ((photon - p->hit->p).squaredLength() <= p->hit->r2) {
            Hit *hp = p->hit;
            // 计算新的光通量信息，并更新当前 Hit
            float factor = (hp->n * ALPHA + ALPHA) / (hp->n * ALPHA + 1.);
            Vector3f dr = d - hp->normal * (2 * Vector3f::dot(d, hp->normal));
            hp->n++;
            hp->r2 *= factor;
            hp->flux = (hp->flux + hp->attenuation * attenuation) * factor;
        }
        // 递归更新左右子树中距离 photon 最近的 Hit 的光通量信息
        if (p->ls) update(p->ls, photon, attenuation, d);
        if (p->rs) update(p->rs, photon, attenuation, d);
        // 更新当前节点所包含的最大半径
        p->maxr2 = p->hit->r2;
        if (p->ls && p->ls->hit->r2 > p->maxr2) p->maxr2 = p->ls->hit->r2;
        if (p->rs && p->rs->hit->r2 > p->maxr2) p->maxr2 = p->rs->hit->r2;
    }

    // 按照 X 轴进行比较
    static bool cmpHitX(Hit *a, Hit *b) { return a->p.x() < b->p.x(); }

    // 按照 Y 轴进行比较
    static bool cmpHitY(Hit *a, Hit *b) { return a->p.y() < b->p.y(); }

    // 按照 Z 轴进行比较
    static bool cmpHitZ(Hit *a, Hit *b) { return a->p.z() < b->p.z(); }
};

#endif
