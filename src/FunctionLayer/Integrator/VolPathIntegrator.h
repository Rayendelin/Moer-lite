#pragma once
#include "Integrator.h"
#include <FunctionLayer/Medium/Medium.h>

// 本地记录，用于统一函数的返回值
struct LocalRecord {
  Vector3f wi;
  Spectrum f;
  float pdf;
  bool isDelta;
};

class VolPathIntegrator : public Integrator {
public:
  VolPathIntegrator() = default;

  VolPathIntegrator(const Json &json);

  virtual Spectrum li(Ray &ray, const Scene &scene,
                      std::shared_ptr<Sampler> sampler) const override;

  // 计算光源上某点到交点的光线透射率
  Spectrum evalTransmittance(const Scene &scene, const Intersection &its, Point3f point, std::shared_ptr<Sampler> sampler) const;

  // 采样一条散射光线
  LocalRecord sampleScatter(const Intersection &its, const Ray &ray, std::shared_ptr<Sampler> sampler) const;

  // 计算发光量，有交点则计算交点作为光源的发光量，无交点则计算环境光
  Spectrum evalEmittance(const Scene &scene, std::optional<Intersection> intersectionOpt, const Ray &ray) const;

  // 计算环境光
  Spectrum evalEnvLight(const Scene &scene, const Ray &ray) const;

  float russianRoulette(int nBounces) const {
    float p = 0.95f;
    if (nBounces > maxDepth)
      p = 0;
    if (nBounces < 2)
      p = 1;
    return p;
  }

  float powerHeuristic(int nf, float pdfF, int ng, float pdfG) const {
    float f = pdfF * nf, g = pdfG * ng;
    return (f * f) / (f * f) + (g * g);
  }

  // 根据介质中的相交信息构造交点
  Intersection fulfillScatterPoint(const Point3f &position, const Vector3f &normal, std::shared_ptr<Medium> medium) const;

  Spectrum uniformSampleOneLight(const Intersection &its, const Scene &scene, const Ray &ray, std::shared_ptr<Sampler> sampler) const;

  Spectrum estimateDirect(const Intersection &its, const Ray &ray, std::shared_ptr<Light> light, const Scene &scene, std::shared_ptr<Sampler> sampler) const;

private:
  uint32_t maxDepth;
};