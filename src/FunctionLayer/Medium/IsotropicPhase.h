#pragma once
#include "PhaseFunction.h"

class IsotropicPhase: public PhaseFunction {
public:
    IsotropicPhase() = default;
    IsotropicPhase(const Json& json): PhaseFunction(json) {}

    virtual float evalPhase(const Vector3f &wo, const Vector3f &wi) const override;
    virtual float samplePhase(const Vector3f &wo, Vector3f *wi, Vector2f& sample) const override;
};

REGISTER_CLASS(IsotropicPhase, "isotropic");