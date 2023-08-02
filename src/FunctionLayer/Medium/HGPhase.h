#pragma once
#include "PhaseFunction.h"

class HGPhase: public PhaseFunction {
public:
    HGPhase(float g): PhaseFunction(), g(g) {}
    HGPhase(const Json& json): PhaseFunction(json) {
        g = fetchRequired<float>(json, "g");
    }

    virtual float evalPhase(const Vector3f &wo, const Vector3f &wi) const override;
    virtual float samplePhase(const Vector3f &wo, Vector3f *wi, Vector2f& sample) const override;

private:
    float g;
};

REGISTER_CLASS(HGPhase, "hg");