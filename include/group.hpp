#ifndef GROUP_H
#define GROUP_H

#include <iostream>
#include <vector>

#include "hit.hpp"
#include "object3d.hpp"
#include "object_kdtree.hpp"
#include "ray.hpp"

// 物体组，相比于原版增加了 KD 树（原版实在太简陋了）
class Group {
   public:
    Group(const vector<Object3D *> &objs) {
        for (auto obj : objs) {
            vector<Object3D *> newFaces = obj->getFaces();// 获取该物体的所有面
            faces.insert(faces.end(), newFaces.begin(), newFaces.end()); // 将面存入列表中
        }
        kdTree = new ObjectKDTree(&faces);// 通过物体列表构建物体KD树
    }
    ~Group() { delete kdTree; }

    // 对列表里所有物体都求一遍交点
    bool intersect(const Ray &r, Hit &h) { return kdTree->intersect(r, h); }// 计算光线与该物体组的交点，并更新命中信息

    bool sequentialSearch(const Ray &r, Hit &h) {
        bool flag = false;
        for (auto face : faces) // 顺序查找每个物体的交点信息
            if (face) flag |= face->intersect(r, h);
        return flag;
    }

    int getGroupSize() { return faces.size(); }
     
    // 重载下标运算符，返回位于i位置的物体
    Object3D *operator[](const int &i) {
        if (i >= faces.size() || i < 0) {
            std::cerr << "Index Error: i = " << i << std::endl;
            return nullptr;
        }
        return faces[i];
    }

    // 获取该物体组中所有光源类型的物体列表
    vector<Object3D *> getIlluminant() const {
        vector<Object3D *> illuminant;
        for (int i = 0; i < faces.size(); ++i)
            if (faces[i]->material->emission != Vector3f::ZERO)// 判断该物体材质是否为发光的
                illuminant.push_back(faces[i]);
        return illuminant;
    }

   private:
    ObjectKDTree *kdTree;
    vector<Object3D *> faces;
};

#endif
