#pragma once
#include <memory>
#include <CoreLayer/Math/Geometry.h>
#include <CoreLayer/ColorSpace/Spectrum.h>
#include <FunctionLayer/Shape/Intersection.h>
#include <FunctionLayer/Sampler/Sampler.h>
#include <ResourceLayer/Factory.h>
#include "PhaseFunction.h"

// 介质采样结果
struct MediumSampleResult {
    float distance;
    float pdf;
    Point3f scatterPoint;
    Vector3f wi;

    Spectrum tr;
    Spectrum sigmaA;
    Spectrum sigmaS;
};

// 介质的抽象类
class Medium {
public:
    Medium(std::shared_ptr<PhaseFunction> phase): mPhase(phase) {}
    Medium(const Json& json) {
        mPhase = Factory::construct_class<PhaseFunction>(json["phase"]);
    }

    virtual bool sampleDistance(MediumSampleResult *mRes, const Ray &ray
        , std::shared_ptr<Sampler> sampler) const = 0;
    virtual Spectrum evalTransmittance(Point3f from, Point3f dest, std::shared_ptr<Sampler> sampler) const = 0;
    
    auto evalPhase(Vector3f wo, Vector3f wi) const {
        return mPhase->evalPhase(wo, wi);
    }

    auto samplePhase(Vector3f wo, Vector3f* wi, Vector2f sample) const {
        return mPhase->samplePhase(wo, wi, sample);
    }

private:
    std::shared_ptr<PhaseFunction> mPhase;  // 相函数，描述介质中发生散射时，散射方向分布的规律
};