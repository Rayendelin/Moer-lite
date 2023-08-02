#pragma once
#include "BSDF.h"
#include "Warp.h"

class OrenNayarBSDF : public BSDF {
public:
  OrenNayarBSDF(const Vector3f &_normal, const Vector3f &_tangent,
                const Vector3f &_bitangent, Spectrum _albedo, float _sigma)
      : BSDF(_normal, _tangent, _bitangent), albedo(_albedo), sigma(_sigma) {}

  static float max(float a, float b) {
    if (a > b)
      return a;
    else
      return b;
  }

  virtual Spectrum f(const Vector3f &wo, const Vector3f &wi) const override {
    // TODO
    // 1. 转换坐标系到局部坐标
    // 2. 计算 A, B, \alpha, \beta（可以直接求\sin\alpha,\tan\beta）,
    // \cos(\phi_i-\phi_o)
    // 3. return Oren-Nayar brdf
    float A = 1 - pow(sigma, 2) / (2 * (pow(sigma, 2) + 0.33));
    float B = 0.45 * pow(sigma, 2) / (pow(sigma, 2) + 0.09);
    Vector3f woLocal = normalize(toLocal(wo));
    Vector3f wiLocal = normalize(toLocal(wi));
    // phi是局部坐标系中向量与x轴的夹角，theta是局部坐标系中向量与y轴的夹角
    Vector3f y = {0, 1.0, 0};
    Vector3f x = {1.0, 0, 0};
    float cos_phi_i = dot(wiLocal, x);
    float cos_phi_o = dot(woLocal, x);
    float sin_phi_i = cross(wiLocal, x).length();
    float sin_phi_o = cross(woLocal, x).length();
    float cos_theta_i = dot(wiLocal, y);
    float cos_theta_o = dot(woLocal, y);
    float sin_theta_i = cross(wiLocal, y).length();
    float sin_theta_o = cross(woLocal, y).length();
    float theta_i = acos(cos_theta_i);
    float theta_o = acos(cos_theta_o);
    float sin_alpha, tan_beta;
    if (theta_o > theta_i) {
      sin_alpha = sin_theta_o;
      tan_beta = sin_theta_i / cos_theta_i;
    }
    else {
      sin_alpha = sin_theta_i;
      tan_beta = sin_theta_o / cos_theta_o;
    }
    return (A + B * max(0, cos_phi_i * cos_phi_o + sin_phi_i * sin_phi_o) * sin_alpha * tan_beta) * albedo / PI;
  }

  virtual BSDFSampleResult sample(const Vector3f &wo,
                                  const Vector2f &sample) const override {
    Vector3f wi = squareToCosineHemisphere(sample);
    float pdf = squareToCosineHemispherePdf(wi);
    return {albedo, toWorld(wi), pdf, BSDFType::Diffuse};
  }

private:
  Spectrum albedo;  // 反照率
  float sigma;  // 粗糙度系数
};