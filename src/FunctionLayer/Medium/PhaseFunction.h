#pragma once
#include <CoreLayer/Math/Geometry.h>
#include <ResourceLayer/Factory.h>
#include <ResourceLayer/JsonUtil.h>

// 相函数的抽象类
class PhaseFunction {
public:
    PhaseFunction() = default;
    PhaseFunction(const Json &json) {};

    virtual float evalPhase(const Vector3f &wo, const Vector3f &wi) const = 0;
    virtual float samplePhase(const Vector3f &wo, Vector3f *wi, Vector2f& sample) const = 0;
};