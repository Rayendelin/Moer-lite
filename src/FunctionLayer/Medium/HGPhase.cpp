#include "HGPhase.h"
#include <CoreLayer/Math/Constant.h>

float HGPhase::evalPhase(const Vector3f& wo, const Vector3f& wi) const {
    float cosTheta = dot(wo, wi) / (wo.length() * wi.length());
    float denom = 1 + g * g + 2 * g * cosTheta;
    return 0.25 * INV_PI * (1 - g * g) / (denom * std::sqrt(denom));
}

float HGPhase::samplePhase(const Vector3f& wo, Vector3f* wi, Vector2f& sample) const {
    float cosTheta;
    if (std::abs(g) < 1e-3)
        cosTheta = 1 - 2 * sample[0];
    else {
        float sqrTerm = (1 - g * g) / (1 - g + 2 * g * sample[0]);
        cosTheta = (1 + g * g - sqrTerm * sqrTerm) / (2 * g);
    }
    float sinTheta = std::sqrt(std::max((float)0, 1 - cosTheta * cosTheta));
    float phi = 2 * PI * sample[1];
    Vector3f v1, v2;
    if (abs(wo[0] > wo[1]))
        v1 = Vector3f(-wo[2], 0, wo[0]) / sqrt(wo[0] * wo[0] + wo[2] * wo[2]);
    else
        v1 = Vector3f(0, wo[2], -wo[1]) / sqrt(wo[1] * wo[1] + wo[2] * wo[2]);
    v2 = cross(wo, v1);
    *wi = v1 * sinTheta * cos(phi) + v2 * sinTheta * sin(phi) + wo * cosTheta;
    float denom = 1 + g * g + 2 * g * cosTheta;
    return 0.25 * INV_PI * (1 - g * g) / (denom * std::sqrt(denom));
}