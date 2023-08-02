#pragma once
#include "BSDF.h"
#include "Warp.h"

class PhongReflection : public BSDF {
public:
  PhongReflection(const Vector3f &_normal, const Vector3f &_tangent,
                  const Vector3f &_bitangent, Spectrum _kd, Spectrum _ks,
                  float _p)
      : BSDF(_normal, _tangent, _bitangent), kd(_kd), ks(_ks), p(_p) {}

  static float max(float a, float b) {
    if (a > b)
      return a;
    else
      return b;
  }

  // 在-wo方向观察到从-wi方向入射的光线的颜色
  virtual Spectrum f(const Vector3f &wo, const Vector3f &wi) const override {
    // TODO
    // 1. 转换坐标系到局部坐标
    // 2. 根据公式计算 K_d, K_s
    // 3. return K_d + K_s
    // tips:
    // Phong模型brdf实现不包括环境光项；其I/r^2项包含在光源采样步骤中，因此brdf中不包含I/r^2。
    Vector3f n = normalize(normal); // 物体表面法线方向
    Vector3f l = normalize(wi);   // 光照入射方向
    Vector3f v = normalize(wo);   // 观察方向
    Spectrum diffuse{0.f};
    Spectrum specular{0.f};
    diffuse = kd * dot(n, l);   // 计算漫反射项系数
    Vector3f lr = l + 2 * (n * dot(n, l) - l);      // 计算光照反射方向lr
    specular = ks * pow(max(0, dot(v, lr)), p); // 计算高光项系数
    return diffuse + specular;
  }

  float pdf(const Vector3f &wo, const Vector3f &wi) const {
    Vector3f woLocal = toLocal(wo), wiLocal = toLocal(wi);
    return squareToCosineHemispherePdf(wiLocal);
  }

  virtual BSDFSampleResult sample(const Vector3f &wo,
                                  const Vector2f &sample) const override {
    Vector3f wiLocal = squareToCosineHemisphere(sample);
    auto wi = toWorld(wiLocal);
    auto bsdf_f = f(wo, wi);
    auto bsdf_pdf = pdf(wo, wi);
    return {bsdf_f / bsdf_pdf, wi, bsdf_pdf, BSDFType::Diffuse};
  }

private:
  Spectrum kd; // 漫反射系数（颜色？）
  Spectrum ks; // 高光（镜面反射）系数（颜色？）
  float p;     // 高光衰减系数
};