#ifndef OBJECTKDTREE_H
#define OBJECTKDTREE_H
#include <vecmath.h> // 向量、矩阵库

#include <map>
#include <vector>

#include "bound.hpp" // AABB包围盒类
#include "hit.hpp" // 求交信息类
#include "object3d.hpp" // 物体基类
using std::map;
using std::vector;

// KD树节点类
class ObjectKDTreeNode {
   public:
    Vector3f min, max; // 节点边界
    vector<Object3D*>* faces; // 指向该节点包含物体的列表
    ObjectKDTreeNode *ls, *rs; // 左右子节点指针
    int l, r; // 该节点所包含物体在整个物体列表中的起止下标
    bool inside(Object3D* face) { // 判断某个物体是否在该节点内
        Vector3f faceMin = face->min(); // 物体最小点
        Vector3f faceMax = face->max(); // 物体最大点
        // 判断是否相交
        return (faceMin.x() < max.x() ||
                faceMin.x() == max.x() && faceMin.x() == faceMax.x()) &&
               (faceMax.x() > min.x() ||
                faceMax.x() == min.x() && faceMin.x() == faceMax.x()) &&
               (faceMin.y() < max.y() ||
                faceMin.y() == max.y() && faceMin.y() == faceMax.y()) &&
               (faceMax.y() > min.y() ||
                faceMax.y() == min.y() && faceMin.y() == faceMax.y()) &&
               (faceMin.z() < max.z() ||
                faceMin.z() == max.z() && faceMin.z() == faceMax.z()) &&
               (faceMax.z() > min.z() ||
                faceMax.z() == min.z() && faceMin.z() == faceMax.z());
    }
};

// KD树类
class ObjectKDTree {
    int n; // 物体数量
    Vector3f** vertices; // 顶点坐标
    // 构建KD树的递归函数
    ObjectKDTreeNode* build(int depth, int d, vector<Object3D*>* faces,
                            const Vector3f& min, const Vector3f& max) {
        ObjectKDTreeNode* p = new ObjectKDTreeNode;
        p->min = min;
        p->max = max;
        Vector3f maxL, minR;
        if (d == 0) { // 按x轴划分
            maxL =
                Vector3f((p->min.x() + p->max.x()) / 2, p->max.y(), p->max.z()); // 左子节点的最大点
            minR =
                Vector3f((p->min.x() + p->max.x()) / 2, p->min.y(), p->min.z()); // 右子节点的最小点
        } else if (d == 1) { // 按y轴划分
            maxL =
                Vector3f(p->max.x(), (p->min.y() + p->max.y()) / 2, p->max.z());
            minR =
                Vector3f(p->min.x(), (p->min.y() + p->max.y()) / 2, p->min.z());
        } else { // 按z轴划分
            maxL =
                Vector3f(p->max.x(), p->max.y(), (p->min.z() + p->max.z()) / 2);
            minR =
                Vector3f(p->min.x(), p->min.y(), (p->min.z() + p->max.z()) / 2);
        }
        p->faces = new vector<Object3D*>;
        for (auto face : *faces)
            if (p->inside(face)) p->faces->push_back(face); // 添加到该节点的物体列表中

        const int max_faces = 128; // 每个节点包含的最大物体数
        const int max_depth = 24; // KD树的最大深度

        if (p->faces->size() > max_faces && depth < max_depth) { // 根据节点中物体数量和深度判断是否需要进一步划分
            p->ls = build(depth + 1, (d + 1) % 3, p->faces, min, maxL); // 构建左子树
            p->rs = build(depth + 1, (d + 1) % 3, p->faces, minR, max); // 构建右子树

            vector<Object3D*>*faceL = p->ls->faces, *faceR = p->rs->faces;
            map<Object3D*, int> cnt; // 记录每个物体被划分到的子树数
            for (auto face : *faceL) cnt[face]++;
            for (auto face : *faceR) cnt[face]++;
            p->ls->faces = new vector<Object3D*>; // 为左子树创建新的物体列表
            p->rs->faces = new vector<Object3D*>; // 为右子树创建新的物体列表
            p->faces->clear(); // 清空该节点原来的物体列表
            for (auto face : *faceL)
                if (cnt[face] == 1)
                    p->ls->faces->push_back(face); // 该物体只被划分到了左子树，加入左子树的物体列表中
                else
                    p->faces->push_back(face); // 该物体被划分到了两个子树，保留在该节点的物体列表中
            for (auto face : *faceR)
                if (cnt[face] == 1) p->rs->faces->push_back(face); // 该物体只被划分到了右子树，加入右子树的物体列表中
        } else
            p->ls = p->rs = nullptr;
        return p;
    }
    // 遍历KD树，获取所有叶子节点包含的物体
    void getFaces(ObjectKDTreeNode* p, vector<Object3D*>* faces) {
        p->l = faces->size();
        for (auto face : *(p->faces)) faces->push_back(face);
        p->r = faces->size();
        if (p->ls) getFaces(p->ls, faces);
        if (p->rs) getFaces(p->rs, faces);
    }

   public:
    ObjectKDTreeNode* root; // KD树的根节点指针
    vector<Object3D*>* faces; // 所有物体的列表
    ObjectKDTree(vector<Object3D*>* faces) { // 构造函数，输入所有物体
        Vector3f min = Vector3f(INF, INF, INF); // 初始化最小点为正无穷
        Vector3f max = -min; // 初始化最大点为负无穷
        for (auto face : *faces) { // 遍历所有物体，更新最小点和最大点
            min = minE(min, face->min());
            max = maxE(max, face->max());
        }
        root = build(1, 0, faces, min, max); // 构建整个KD树
        this->faces = new vector<Object3D*>; // 创建新的物体列表
        getFaces(root, this->faces); // 获取KD树中所有叶子节点包含的物体，并将其加入该对象的物体列表中
    }

    float cuboidIntersect(ObjectKDTreeNode* p, const Ray& ray) const { // 计算光线与该节点对应的包围盒的交点
        float t = INF;
        if (!p) return t; // 如果该节点为空，则返回正无穷
        AABB(p->min, p->max).intersect(ray, t); // 计算光线与包围盒的交点
        return t;
    }

    bool intersect(const Ray& ray, Hit& hit) const { // 判断光线是否与该KD树中的物体相交
        Object3D* nextFace = nullptr; // 下一个可能相交的物体
        return intersect(root, ray, nextFace, hit); // 递归判断
    }

    bool intersect(ObjectKDTreeNode* p, const Ray& ray, Object3D*& nextFace,
               Hit& hit) const {
    bool flag = false; // 是否有交点的标志
    for (int i = 0; i < p->faces->size(); ++i) // 遍历该节点中所有物体
        if ((*p->faces)[i]->intersect(ray, hit)) { // 如果光线与物体相交
            nextFace = (*p->faces)[i]; // 更新下一个可能相交的物体为该物体
            flag = true; // 设置交点标志为true
        }
    float tl = cuboidIntersect(p->ls, ray), // 计算光线与左子树包围盒的交点
          tr = cuboidIntersect(p->rs, ray); // 计算光线与右子树包围盒的交点
    if (tl < tr) { // 如果左子树的包围盒更靠近光线
        if (hit.t <= tl) return flag; // 如果左子树对应的包围盒距离比当前最优的交点距离还远，则直接返回
        if (p->ls) flag |= intersect(p->ls, ray, nextFace, hit); // 递归遍历左子树
        if (hit.t <= tr) return flag; // 如果右子树对应的包围盒距离比当前最优的交点距离还远，则直接返回
        if (p->rs) flag |= intersect(p->rs, ray, nextFace, hit); // 递归遍历右子树
    } else { // 如果右子树的包围盒更靠近光线
        if (hit.t <= tr) return flag; // 如果右子树对应的包围盒距离比当前最优的交点距离还远，则直接返回
        if (p->rs) flag |= intersect(p->rs, ray, nextFace, hit); // 递归遍历右子树
        if (hit.t <= tl) return flag; // 如果左子树对应的包围盒距离比当前最优的交点距离还远，则直接返回
        if (p->ls) flag |= intersect(p->ls, ray, nextFace, hit); // 递归遍历左子树
    }
    return flag; // 返回是否有交点的标志
}

};

#endif  // !OBJECTKDTREE_H