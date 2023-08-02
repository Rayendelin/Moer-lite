#include "IsotropicPhase.h"
#include <CoreLayer/Math/Constant.h>

float IsotropicPhase::evalPhase(const Vector3f& wo, const Vector3f& wi) const {
    return 0.25 * INV_PI;
}

float IsotropicPhase::samplePhase(const Vector3f& wo, Vector3f* wi, Vector2f& sample) const {
    float z = 1 - 2 * sample[0];
    float r = std::sqrt(std::max((float)0, 1 - z * z));
    float phi = 2 * PI * sample[1];
    *wi = Vector3f(r * cos(phi), r * sin(phi), abs(z));
    return 0.25 * INV_PI;
}