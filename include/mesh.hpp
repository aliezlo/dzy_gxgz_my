#ifndef MESH_H
#define MESH_H

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <utility>
#include <vector>

#include "Vector2f.h"
#include "Vector3f.h"
#include "bound.hpp"
#include "object3d.hpp"
#include "object_kdtree.hpp"
#include "ray.hpp"
#include "triangle.hpp"
#include "utils.hpp"
class Mesh : public Object3D {
   public:

   struct TriangleIndex {
        TriangleIndex() {
            x[0] = -1;
            x[1] = -1;
            x[2] = -1;
        }
        int &operator[](const int i) { return x[i]; }
        // By Computer Graphics convention, counterclockwise winding is front
        // face
        int x[3]{};
        bool valid() { return x[0] != -1 && x[1] != -1 && x[2] != -1; }
    };

    // 从 OBJ 文件中读取顶点、贴图和法向量信息
void readObjFile(const char *filename, std::vector<TriangleIndex> &vIdx, std::vector<Vector3f> &v,
                 std::vector<Vector2f> &vt, std::vector<Vector3f> &vn, AABB &aabb) {
    std::ifstream f;
    f.open(filename);
    if (!f.is_open()) {
        std::cout << "Cannot open " << filename << "\n";
        return;
    }
    // 定义一些字符串常量，用于从每行中提取信息
    std::string line;
    std::string vTok("v");
    std::string fTok("f");
    std::string vnTok("vn");
    std::string texTok("vt");
    std::string bslash("/");
    std::string space(" ");
    std::string tok;
    int texID;

    while (true) { // 循环逐行读取 OBJ 文件
        std::getline(f, line); // 读取一行
        if (f.eof()) { // 如果已到达 EOF，退出循环
            break;
        }
        if (line.size() < 3) { // 如果读取到的字符串长度小于三个字符，跳过，因为不可能包含有意义的信息
            continue;
        }
        if (line.at(0) == '#') { // 如果该行以“#”开头，跳过，因为该行是注释
            continue;
        }
        std::stringstream ss(line);
        ss >> tok; // 读取该行第一个字符串
        if (tok == vTok) { // 如果是“v”，说明该行包含顶点信息
            Vector3f vec;
            ss >> vec[0] >> vec[1] >> vec[2];
            v.push_back(vec); // 将该行的顶点信息存入向量中
            aabb.updateBound(vec); // 更新 AABB 包围盒
        } else if (tok == fTok) { // 如果是“f”，说明该行包含三角形面信息
            bool tFlag = 1, nFlag = 1;
            TriangleIndex vId, tId, nId;
            for (int i = 0; i < 3; ++i) { // 逐个读取三角形面的每一个顶点信息
                std::string str;
                ss >> str;
                std::vector<std::string> id = split(str, bslash); // 提取顶点、贴图和法向量索引
                vId[i] = atoi(id[0].c_str()) - 1; // 将字符串转换为整数并减1的目的是将索引从1-based变为0-based
                if (id.size() > 1) {
                    tId[i] = atoi(id[1].c_str()) - 1;
                }
                if (id.size() > 2) {
                    nId[i] = atoi(id[2].c_str()) - 1;
                }
            }
            vIdx.push_back(vId); // 将该三角形面的顶点索引存入向量中
        } else if (tok == texTok) { // 如果是“vt”，说明该行包含贴图坐标信息
            Vector2f texcoord;
            ss >> texcoord[0];
            ss >> texcoord[1];
            vt.push_back(texcoord); // 将该行的贴图坐标存入向量中
        } else if (tok == vnTok) { // 如果是“vn”，说明该行包含法向量信息
            Vector3f vec;
            ss >> vec[0] >> vec[1] >> vec[2];
            vn.push_back(vec); // 将该行的法向量信息存入向量中
        }
    }
    f.close(); // 关闭文件
}

// 根据从 OBJ 文件读取到的数据创建三角形面并存储到向量 triangles 中
void createTriangles(std::vector<Object3D *> &triangles, std::vector<TriangleIndex> &vIdx,
                     std::vector<Vector3f> &v, std::vector<Vector2f> &vt, std::vector<Vector3f> &vn, Material *m) {
    for (int triId = 0; triId < (int)vIdx.size(); ++triId) { // 循环遍历所有三角形面并存储到 triangles 向量中
        TriangleIndex &vIndex = vIdx[triId];
        triangles.push_back((Object3D *)new Triangle(
            v[vIndex[0]], v[vIndex[1]], v[vIndex[2]], m)); // 将三角形面的三个顶点坐标和材质参数传入 Triangle 类的构造函数创建一个三角形，并将其转换为 Object3D* 类型存入 triangles 向量中
        TriangleIndex &tIndex = vIdx[triId];
        if (tIndex.valid() && !vt.empty())
            ((Triangle *)triangles.back())
                    ->setVT(vt[tIndex[0]], vt[tIndex[1]], vt[tIndex[2]]); // 如果该三角形面有贴图信息，将其传入 Triangle 类的 setVT() 函数中进行处理
        TriangleIndex &nIndex = vIdx[triId];
        if (nIndex.valid() && !vn.empty())
            ((Triangle *)triangles.back())
                    ->setVNorm(vn[nIndex[0]], vn[nIndex[1]], vn[nIndex[2]]); // 如果该三角形面有法向量信息，将其传入 Triangle 类的 setVNorm() 函数中进行处理
    }
}

// Mesh 构造函数，需要传入 OBJ 文件路径和材质参数
Mesh(const char *filename, Material *m) : Object3D(m) {
    std::vector<TriangleIndex> vIdx; // 存储顶点、贴图和法向量索引的向量
    std::vector<Vector3f> v, vn; // 存储顶点和法向量的向量
    std::vector<Vector2f> vt; // 存储贴图坐标的向量
    

    readObjFile(filename, vIdx, v, vt, vn, aabb); // 从 OBJ 文件中读取数据
    createTriangles(triangles, vIdx, v, vt, vn, m); // 根据读取到的数据创建三角形面

    kdTree = new ObjectKDTree(&triangles); // 创建 kd-tree
}


    ~Mesh() {
        for (int i = 0; i < triangles.size(); ++i) delete triangles[i];
        delete kdTree;
    }

    // 从给定字符串中按照指定模式拆分出多个子字符串
    std::vector<std::string> split(std::string str, std::string pattern) {
        std::string::size_type pos;
        std::vector<std::string> result;
        // 将待拆分的字符串末尾添加模式字符串，目的是保证最后一个子串一定能够被提取出来
        str += pattern;
        int size = str.size();

        for (int i = 0; i < size; i++) {
            pos = str.find(pattern, i); // 从当前位置开始查找模式字符串
            if (pos < size) { // 如果找到了模式字符串
                // 提取从 i 到 pos 之间的子串，并将其存入结果向量中
                std::string s = str.substr(i, pos - i);
                result.push_back(s);
                i = pos + pattern.size() - 1; // 更新 i 为下一个要查找的位置
            }
        }
        return result; // 返回拆分后的字符串向量
    }

    
    vector<Object3D *> triangles;
    bool intersect(const Ray &r, Hit &h) override {
        float tb;
        //如果光线与包围盒不相交，返回 false
        if (!aabb.intersect(r, tb)) return false;
        //如果光线与包围盒相交，但是交点距离比当前最近交点还要远，返回 false
        if (tb > h.getT()) return false;
        //kd-tree 加速求交
        bool flag = kdTree->intersect(r, h);
        return flag;
    }

    // 顺序求交
    bool sequentialSearch(const Ray &r, Hit &h) {
        bool result = false;
        for (auto triangle : triangles) result |= triangle->intersect(r, h);
        return result;
    }

    Vector3f min() const override { return aabb.bounds[0]; }
    Vector3f max() const override { return aabb.bounds[1]; }
    Vector3f center() const override {
        return (aabb.bounds[0] + aabb.bounds[1]) / 2;
    }
    
    //返回所有三角形面片
    vector<Object3D *> getFaces() override { return {(Object3D *)this}; }
    
    // randomRay() 函数用于随机生成一条光线,axis为光线的方向
    Ray randomRay(int axis=-1, long long int seed=0) const override {
        int trig = random(axis, seed) * triangles.size();
        return triangles[trig]->randomRay(axis, seed);
    }

   private:
    AABB aabb;
    ObjectKDTree *kdTree;
};

#endif
