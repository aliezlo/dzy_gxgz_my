#ifndef PATH_TRACER_H
#define PATH_TRACER_H

#include <cmath>
#include <cstring>
#include <iostream>
#include <string>

#include "camera.hpp"
#include "constants.h"
#include "group.hpp"
#include "hit.hpp"
#include "hit_kdtree.hpp"
#include "image.hpp"
#include "light.hpp"
#include "ray.hpp"
#include "scene.hpp"
#include "utils.hpp"
using namespace std;


class SPPM {
   public:
    const Scene& scene; // 场景
    int numRounds; // 迭代次数
    int numPhotons; // 光子数
    int ckpt_interval; // 保存检查点的间隔迭代次数
    std::string outdir; // 输出路径
    int w, h; // 图像宽高
    Camera* camera; // 相机
    vector<Hit*> hitPoints; // hit points
    HitKDTree* hitKDTree; // hit points 的 kd 树
    vector<Object3D*> illuminants; // 发光物体
    Group* group; // 物体组
    SPPM(const Scene& scene, int numRounds, int numPhotons, int ckpt,
         const char* dir)
        : scene(scene),
          numRounds(numRounds),
          numPhotons(numPhotons),
          ckpt_interval(ckpt),
          outdir(dir) {
        hitKDTree = nullptr;
        // 获取相机和物体组与发光物体
        camera = scene.getCamera();
        group = scene.getGroup();
        illuminants = group->getIlluminant();
        // 获取场景宽高
        w = camera->getWidth();
        h = camera->getHeight();

        // 初始化 hitPoints
        for (int u = 0; u < w; ++u)
            for (int v = 0; v < h; ++v) hitPoints.push_back(new Hit());
        // 输出场景宽高
        cout << "Width: " << w << " Height: " << h << endl;
    }

    ~SPPM() {
        // 销毁 hitPoints 和 hitKDTree
        for (int u = 0; u < w; ++u)
            for (int v = 0; v < h; ++v) delete hitPoints[u * w + v];
        delete hitKDTree;
    }

    void forward(Ray ray, Hit* hit) {
    // 深度与衰减系数
    int depth = 0;
    Vector3f attenuation(1, 1, 1);
    while (true) {
        // 终止条件：达到追踪深度或衰减系数较小
        if (++depth > TRACE_DEPTH || attenuation.max() < 1e-3) return;
        hit->t = INF;
        // 射线与场景中物体求交
        if (!group->intersect(ray, *hit)) {
            // 未与场景中的物体相交则将背景色添加到 hit 的发光通量中
            hit->fluxLight += hit->attenuation*scene.getBackgroundColor();
            return;
        }
        // 更新射线起点并计算当前物体表面反射和透射率
        ray.origin += ray.direction * (*hit).t;
        Material* material = (*hit).material;
        Vector3f N(hit->normal);
        float type = RND2;
        // 根据物体类型进行处理
        if (type <= material->type.x()) {  // 漫反射
            // 将 hit 点与相应的贡献存储到 hit point 中
            hit->attenuation = attenuation * hit->color;
            hit->fluxLight += hit->attenuation * material->emission;
            return;
        } else if (type <= material->type.x() + material->type.y()) { // 镜面反射
            // 计算反射方向
            float cost = Vector3f::dot(ray.direction, N);
            ray.direction = (ray.direction - N * (cost * 2)).normalized();
        } else { // 折射
            float n = material->refr;
            float R0 = ((1.0 - n) * (1.0 - n)) / ((1.0 + n) * (1.0 + n));
            if (Vector3f::dot(N, ray.direction) > 0) {  // 入射面在介质内部
                N.negate();
                n = 1 / n;
            }
            n = 1 / n;
            float cost1 =
                -Vector3f::dot(N, ray.direction);  // 入射角
            float cost2 =
                1.0 - n * n * (1.0 - cost1 * cost1);  // 折射角
            float Rprob =
                R0 + (1.0 - R0) * pow(1.0 - cost1,
                                      5.0);   // Schlick-approximation，计算反射系数
            if (cost2 > 0 && RND2 > Rprob) {  // 折射方向
                ray.direction =
                    ((ray.direction * n) + (N * (n * cost1 - sqrt(cost2))))
                        .normalized();
            } else {  // 反射方向
                ray.direction =
                    (ray.direction + N * (cost1 * 2)).normalized();
            }
        }
        // 更新衰减系数，即当前区域所受到的颜色衰减情况
        attenuation = attenuation * hit->color;
    }
}


    void backward(Ray ray, const Vector3f& color, long long int seed=-1) {
        // 深度与衰减系数
        int depth = 0;
        Vector3f attenuation = color * Vector3f(250, 250, 250);
        while (true) {
            // 终止条件：达到追踪深度或衰减系数较小
            if (++depth > TRACE_DEPTH || attenuation.max() < 1e-3) return;
            // 射线与场景中物体求交
            Hit hit;
            if (!group->intersect(ray, hit)) return;
            ray.origin += ray.direction * hit.t;
            Material* material = hit.material;
            Vector3f N(hit.normal);
            float type = RND2;
            //  根据物体类型进行处理
            //  漫反射
            if (type <= material->type.x()) {  
                // 将 hit 点与相应的贡献存储到 hit point 中
                hitKDTree->update(hitKDTree->root, hit.p, attenuation,
                                  ray.direction);
                // 生成新的射线，方向为半球随机方向
                ray.direction = diffDir(N, -1, seed);
            } else if (type <= material->type.x() + material->type.y()) {
                // 镜面反射
                float cost = Vector3f::dot(ray.direction, N);
                ray.direction = (ray.direction - N * (cost * 2)).normalized();
            } else {
                // 折射
                float n = material->refr;
                float R0 = ((1.0 - n) * (1.0 - n)) / ((1.0 + n) * (1.0 + n));
                if (Vector3f::dot(N, ray.direction) > 0) {  
                    N.negate();
                    n = 1 / n;
                }
                n = 1 / n;
                // 计算反射系数
                float cost1 =
                    -Vector3f::dot(N, ray.direction);  
                float cost2 =
                    1.0 - n * n * (1.0 - cost1 * cost1);  
                float Rprob =
                    R0 + (1.0 - R0) * pow(1.0 - cost1,
                                          5.0);  
                // 根据反射系数判断反射或折射
                if (cost2 > 0 && RND2 > Rprob) {  
                    // 计算折射方向
                    ray.direction =
                        ((ray.direction * n) + (N * (n * cost1 - sqrt(cost2))))
                            .normalized();
                } else {  // 计算反射方向
                    ray.direction =
                        (ray.direction + N * (cost1 * 2)).normalized();
                }
            }
            attenuation = attenuation * hit.color;
        }
    }

    void render() {
    time_t start = time(NULL);
    Vector3f color = Vector3f::ZERO;
    // 进行多轮迭代
    for (int round = 0; round < numRounds; ++round) {
        float elapsed = (time(NULL) - start),
              progress = (1. + round) / numRounds;
        // 打印当前迭代的进度信息
        fprintf(stderr,
                "\rRendering (%d/%d Rounds) %5.2f%% Time: %.2f/%.2f sec\n",
                round + 1, numRounds, progress * 100., elapsed,
                elapsed / progress);
#pragma omp parallel for schedule(dynamic, 1)
        // 对每个像素点进行前向追踪计算
        for (int x = 0; x < w; ++x) {
            for (int y = 0; y < h; ++y) {
                Ray camRay =
                    camera->generateRay(Vector2f(x + RND, y + RND));
                hitPoints[x * h + y]->reset(-camRay.direction);
                forward(camRay, hitPoints[x * h + y]);
            }
        }
        // 将 hitPoints 存储到 KD 树中
        setHitKDTree();
        int photonsPerLight = numPhotons / illuminants.size();
        // 对每个发光物体，随机生成一定数量的光子，并对其进行后向追踪计算
#pragma omp parallel for schedule(dynamic, 1)
        for (int i = 0; i < photonsPerLight; ++i) {
            for (int j = 0;j < illuminants.size(); ++j) {
                Ray ray = illuminants[j]->randomRay(-1, (long long)round * numPhotons + (round + 1) * w * h + i);
                backward(ray, illuminants[j]->material->emission, (long long)round * numPhotons + i);
            } 
        }
        // 每隔一定迭代次数保存检查点
        if ((round + 1) % ckpt_interval == 0) {
            char filename[100];
            sprintf(filename, "ckpt-%d.bmp", round + 1);
            save(filename, round + 1, numPhotons);
        }
    }
    // 将最终计算的图像输出到文件
    save("result.bmp", numRounds, numPhotons);
    }


    void save(std::string filename, int numRounds, int numPhotons) {
    // 将 hitPoints 中的贡献输出到图像中
        Image outImg(w, h);
        for (int u = 0; u < w; ++u)
            for (int v = 0; v < h; ++v) {
                Hit* hit = hitPoints[u * h + v];
                outImg.SetPixel(
                    u, v,
                    hit->flux / (M_PI * hit->r2 * numPhotons * numRounds) +
                        hit->fluxLight / numRounds);
            }
        outImg.SaveBMP((outdir + "/" + filename).c_str());
    }

    void setHitKDTree() {
        if (hitKDTree) delete hitKDTree;
        hitKDTree = new HitKDTree(&hitPoints);
    }
};

#endif  // !PATH_TRACER_H