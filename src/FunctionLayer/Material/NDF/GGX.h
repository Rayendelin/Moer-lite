#pragma once

#include "NDF.h"

class GGXDistribution : public NDF {
public:
  GGXDistribution() noexcept = default;
  virtual ~GGXDistribution() noexcept = default;
  virtual float getD(const Vector3f &whLocal,
                     const Vector2f &alpha) const noexcept override {
    // TODO
    // 根据公式即可
    // 这里假设传入的whLocal已经归一化
    Vector3f y = {0, 1.0, 0};
    float cos_theta = dot(whLocal, y);
    float sin_theta = cross(whLocal, y).length();
    float tan_theta = sin_theta / cos_theta;
    return pow(alpha[0], 2) / (PI * pow(cos_theta, 4) * pow(pow(alpha[0], 2) + pow(tan_theta, 2), 2));
  }

  // tips:
  // float getG1(...) {}
  float getG1(const Vector3f& wLocal, const Vector2f& alpha) const {
    // theta是光线方向与宏观法线的夹角
    Vector3f y = {0, 1.0, 0};
    float tan_theta = cross(wLocal, y).length() / dot(wLocal, y);
    return 2 / (1 + sqrt(1 + pow(alpha[0], 2) * pow(tan_theta, 2)));
  }

  virtual float getG(const Vector3f &woLocal, const Vector3f &wiLocal,
                     const Vector2f &alpha) const noexcept override {
    // TODO
    // 根据公式即可
    // tips: return getG1(wo) * getG1(wi);
    return getG1(woLocal, alpha) * getG1(wiLocal, alpha);
  }
  virtual float pdf(const Vector3f &woLocal, const Vector3f &whLocal,
                    const Vector2f &alpha) const noexcept override {
    return getD(whLocal, alpha) * whLocal[1];
  }
  virtual Vector3f sampleWh(const Vector3f &woLocal, const Vector2f &alpha,
                            const Vector2f &sample) const noexcept override {
    float a = alpha[0];
    float tan_theta_2 = a * a * sample[0] / (1.f - sample[0]);
    float phi = sample[1] * 2 * PI;

    float cos_theta = std::sqrt(1.f / (1.f + tan_theta_2));
    float sin_theta = std::sqrt(1.f - cos_theta * cos_theta);
    return {sin_theta * std::cos(phi), sin_theta * std::sin(phi), cos_theta};
  }
};