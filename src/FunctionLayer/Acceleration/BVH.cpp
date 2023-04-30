#include "BVH.h"

struct BVH::BVHNode {
    //* todo BVH节点结构设计
    BVHNode* left;
    BVHNode* right;
    AABB box;   // 非叶子节点的节点包围盒
    int firstShapeOffset;   // 叶子节点中第一个物体的索引
    int nShape = 0; // 叶子节点存放的物体数量
    int splitAxis = 0;  // 非叶子节点的分割轴，0表示X轴，1表示Y轴，2表示Z轴
    BVHNode() { box = AABB(); left = nullptr; right = nullptr; }
};

// 递归构建节点，在调用该函数之前可以先为node填充好分割轴参数
void BVH::recursiveBuild(BVHNode* node, int begin, int end) {
    // 根据当前节点分割轴对当前节点囊括的物体进行从小到大排序
    int splitAxis = node->splitAxis;
    std::sort(shapes.begin() + begin, shapes.begin() + end, [&](std::shared_ptr<Shape> a, std::shared_ptr<Shape> b)
        { return a->getAABB().Center()[splitAxis] < b->getAABB().Center()[splitAxis]; });
    // 构建叶子节点
    if (end - begin <= bvhLeafMaxSize) {
        node->firstShapeOffset = begin;
        node->nShape = end - begin;
        for (auto it = shapes.begin() + begin; it != shapes.begin() + end; it++)
            node->box.Expand((*it)->getAABB());
        return;
    }
    //递归构建非叶子节点的左右节点
    BVHNode* left = new BVHNode();
    BVHNode* right = new BVHNode();
    int num = (end - begin) / 2;
    this->recursiveBuild(left, begin, begin + num);
    this->recursiveBuild(right, begin + num, end);
    node->left = left;
    node->right = right;
    node->box.Expand(left->box);
    node->box.Expand(right->box);
    return;
}

void BVH::build() {
    // AABB sceneBox;
    // shapes定义在BVH的父类Acceleration中
    for (const auto & shape : shapes) {
        //* 自行实现的加速结构请务必对每个shape调用该方法，以保证TriangleMesh构建内部加速结构
        //* 由于使用embree时，TriangleMesh::getAABB不会被调用，因此出于性能考虑我们不在TriangleMesh
        //* 的构造阶段计算其AABB，因此当我们将TriangleMesh的AABB计算放在TriangleMesh::initInternalAcceleration中
        //* 所以请确保在调用TriangleMesh::getAABB之前先调用TriangleMesh::initInternalAcceleration
        shape->initInternalAcceleration();
        // sceneBox.Expand(shape->getAABB());
    }
    //* todo 完成BVH构建
    root = new BVHNode();
    this->recursiveBuild(root, 0, shapes.end() - shapes.begin());
}

void BVH::recursiveRayIntersect(BVHNode* node, Ray &ray, int *geomID, int *primID, float *u, float *v) const {
    float tMin, tMax;
    bool currFlag = node->box.RayIntersect(ray, &tMin, &tMax);
    // 光不与当前node的包围盒相交
    if (!currFlag || (tMax < ray.tNear || tMin > ray.tFar))
        return;
    int splitAxis = node->splitAxis;
    // 如果node是叶子节点
    if (node->nShape > 0) {
        bool flag = false;
        if (ray.direction[splitAxis] >= 0) {
            // 遍历叶子节点中的物体
            for (int i = node->firstShapeOffset; i < node->firstShapeOffset + node->nShape; i++) {
                float tMin, tMax;
                bool currFlag = shapes[i]->getAABB().RayIntersect(ray, &tMin, &tMax);
                // 光不与当前物体的包围盒相交
                if (!currFlag || (tMax < ray.tNear || tMin > ray.tFar))
                    continue;
                if (shapes[i]->rayIntersectShape(ray, primID, u, v))
                    *geomID = shapes[i]->geometryID;
            }
        }
        else {
            for (int i = node->firstShapeOffset + node->nShape - 1; i >= node->firstShapeOffset; i--) {
                float tMin, tMax;
                bool currFlag = shapes[i]->getAABB().RayIntersect(ray, &tMin, &tMax);
                if (!currFlag || (tMax < ray.tNear || tMin > ray.tFar))
                    continue;
                if (shapes[i]->rayIntersectShape(ray, primID, u, v))
                    *geomID = shapes[i]->geometryID;
            }
        }
        return;
    }
    // 光在分割轴上的方向为正方向，所以先检查光是否与左节点相交
    if (ray.direction[splitAxis] >= 0) {
        this->recursiveRayIntersect(node->left, ray, geomID, primID, u, v);
        this->recursiveRayIntersect(node->right, ray, geomID, primID, u, v);
    }
    else {
        this->recursiveRayIntersect(node->right, ray, geomID, primID, u, v);
        this->recursiveRayIntersect(node->left, ray, geomID, primID, u, v);
    }
    return;
}

bool BVH::rayIntersect(Ray &ray, int *geomID, int *primID, float *u, float *v) const {
    //* todo 完成BVH求交
    this->recursiveRayIntersect(root, ray, geomID, primID, u, v);
    return (*geomID != -1);
}


