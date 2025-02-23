//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_BVH_H
#define RAYTRACING_BVH_H

#include <atomic>
#include <vector>
#include <memory>
#include <ctime>
#include "Object.hpp"
#include "Ray.hpp"
#include "Bounds3.hpp"
#include "Intersection.hpp"
#include "Vector.hpp"

// BVHBuildNode 前向声明
struct BVHBuildNode;
// BVHAccel Forward Declarations 前向声明
struct BVHPrimitiveInfo;

// BVHAccel Declarations
inline int leafNodes, totalLeafNodes, totalPrimitives, interiorNodes;

// BVH加速结构体
class BVHAccel {

public:
    // BVHAccel Public Types 枚举类型
    // 分割方法
    enum class SplitMethod { NAIVE, SAH };

    // BVHAccel Public Methods 公有方法
    
    // 构造函数
    /*
    参数：
    p: 物体列表
    maxPrimsInNode: 最大物体数量
    splitMethod: 分割方法
    */
    BVHAccel(std::vector<Object*> p, int maxPrimsInNode = 1, SplitMethod splitMethod = SplitMethod::NAIVE);
    // 析构函数
    ~BVHAccel();

    // 获取包围盒
    Bounds3 WorldBound() const;
    
    // 光线在 BVH树中碰撞
    Intersection Intersect(const Ray &ray) const; // 光线在 BVH树中碰撞
    // 求得碰撞信息
    Intersection getIntersection(BVHBuildNode* node, const Ray& ray) const; // 求得碰撞信息

    bool IntersectP(const Ray &ray) const;

    BVHBuildNode* root; // BVH树根节点

    // BVHAccel Private Methods 私有方法
    BVHBuildNode* recursiveBuild(std::vector<Object*>objects);

    // BVHAccel Private Data 私有数据
    const int maxPrimsInNode;
    const SplitMethod splitMethod;
    std::vector<Object*> primitives;
};

// BVHBuildNode 结构体
/*
参数：
Bounds3 bounds; // 包围盒，两个三维空间点
BVHBuildNode *left; // 左子节点
BVHBuildNode *right; // 右子节点
Object* object; // 包围盒中的物体
*/
struct BVHBuildNode {
    Bounds3 bounds; // 包围盒，两个三维空间点
    BVHBuildNode *left;
    BVHBuildNode *right;
    Object* object; // 包围盒中的物体

public:
    // 初始化分割轴，第一个物体偏移量，物体数量
    int splitAxis=0, firstPrimOffset=0, nPrimitives=0;
    // BVHBuildNode Public Methods 
    // 构造函数
    BVHBuildNode(){
        bounds = Bounds3();
        left = nullptr;
        right = nullptr;
        object = nullptr;
    }
};




#endif //RAYTRACING_BVH_H
