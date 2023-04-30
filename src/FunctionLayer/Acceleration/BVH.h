#pragma once
#include "Acceleration.h"
#include <algorithm>

class BVH : public Acceleration{
protected:
    struct BVHNode;
    static constexpr int bvhLeafMaxSize = 5;
    BVHNode* root;
public:
    BVH() = default;
    void build() override;
    void recursiveBuild(BVHNode* node, int begin, int end);
    bool rayIntersect(Ray &ray, int *geomID, int *primID, float *u, float *v) const override;
    void recursiveRayIntersect(BVHNode* node, Ray &ray, int *geomID, int *primID, float *u, float *v) const;
};