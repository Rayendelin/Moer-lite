#pragma once
#include "../NDF/NDF.h"
#include "BSDF.h"
#include "Warp.h"

// Conductor导体
class RoughConductorBSDF : public BSDF {
public:
  RoughConductorBSDF(const Vector3f &_normal, const Vector3f &_tangent,
                     const Vector3f &_bitangent, Spectrum _albedo,
                     Vector2f _alpha, Vector3f _eta, Vector3f _k, NDF *_ndf)
      : BSDF(_normal, _tangent, _bitangent), albedo(_albedo), alpha(_alpha),
        eta(_eta), k(_k), ndf(_ndf) {}

  virtual Spectrum f(const Vector3f &wo, const Vector3f &wi) const override {
    // TODO
    // 1. 转换坐标系到局部坐标
    // 2. 根据公式计算 Fr, D, G
    // 3. return albedo * D * G * Fr / (4 * \cos\theta_o);
    // tips: brdf
    // 中分母的\cos\theta_i项被渲染方程中的cos项消去，因此分母只有4*\cos\theta_o
    Vector3f woLocal = normalize(toLocal(wo));
    Vector3f wiLocal = normalize(toLocal(wi));
    Vector3f whLocal = normalize(woLocal + wiLocal);
    float D = ndf->getD(whLocal, alpha);
    float G = ndf->getG(woLocal, wiLocal, alpha);
    float cos_theta = dot(normalize(wi), normalize(normal));
    Vector3f fr = getFr(cos_theta);
    float cos_theta_o = dot(normalize(wo), normalize(normal));
    return albedo * D * G * fr[0] / (4 * cos_theta_o);
  }

  virtual BSDFSampleResult sample(const Vector3f &wo,
                                  const Vector2f &sample) const override {
    Vector3f wi = squareToCosineHemisphere(sample);
    float pdf = squareToCosineHemispherePdf(wi);
    return {f(wo, toWorld(wi)) / pdf, toWorld(wi), pdf, BSDFType::Diffuse};
  }

  Vector3f getR0() const noexcept {
    return ((eta - 1.f) * (eta - 1.f) + k * k) /
           ((eta + 1.f) * (eta + 1.f) + k * k);
  }
  Vector3f getFr(float cosTheta) const noexcept {
    Vector3f r0 = getR0();
    return r0 + (Vector3f(1.f) - r0) * std::pow(1.f - cosTheta, 5.f);
  }

private:
  Spectrum albedo;
  Vector2f alpha;
  Vector3f eta;
  Vector3f k;
  NDF *ndf;
};