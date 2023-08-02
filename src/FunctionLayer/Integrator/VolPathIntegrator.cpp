#include "VolPathIntegrator.h"
#include <FunctionLayer/Material/BxDF/Warp.h>
#include <FunctionLayer/Material/Matte.h>

Spectrum VolPathIntegrator::li(Ray &ray, const Scene &scene,
                            std::shared_ptr<Sampler> sampler) const {
  const float eps = 1e-4;
  Spectrum spectrum(.0f);
  Spectrum throughput(1.f);
  bool specularBounce = false;
  int nBounces = 0;

  for (nBounces = 0;; nBounces++) {
    auto intersectionOpt = scene.rayIntersect(ray);   // 求光线与场景中物体的交点
    
    MediumSampleResult mRes;
    bool sampleMedium = false;
    if (ray.medium.get() != nullptr)  // 如果光线的起点正处于介质中，那么采样可能在介质中发生的碰撞
    {
      sampleMedium = ray.medium->sampleDistance(&mRes, ray, sampler);
      throughput *= sampleMedium ? (mRes.tr * mRes.sigmaS / mRes.pdf) : (mRes.tr / mRes.pdf);
    }

    if (throughput.isZero())
      break;

    // 处理介质中发生的碰撞
    if (sampleMedium) {
      if (nBounces >= maxDepth)
        break;
      
      Intersection mediumScatterPoint = fulfillScatterPoint(mRes.scatterPoint, ray.direction, ray.medium);

      spectrum += throughput * uniformSampleOneLight(mediumScatterPoint, scene, ray, sampler);

      // 采样一条散射光线
      LocalRecord sampleScatterRecord = sampleScatter(mediumScatterPoint, ray, sampler);
      ray = Ray(mediumScatterPoint.position + sampleScatterRecord.wi * eps, sampleScatterRecord.wi, eps, 1e10f, .0f,
        mediumScatterPoint.getMedium(sampleScatterRecord.wi));
      specularBounce = false;
    }
    // 处理物体表面发生的碰撞
    else {
      // 考虑交点处的发光
      if (nBounces == 0 || specularBounce)
        spectrum += evalEmittance(scene, intersectionOpt, ray);
      
      if (!intersectionOpt.has_value() || nBounces >= maxDepth)
        break;

      Intersection intersection = intersectionOpt.value();
      computeRayDifferentials(&intersection, ray);

      // 如果是介质表面
      if (!intersection.isSurfaceIts()) {
        nBounces--;
        ray = Ray(intersection.position + ray.direction * eps, ray.direction, eps, 1e10f, .0f,
          intersection.getMedium(ray.direction));
        continue;
      }

      spectrum += throughput * uniformSampleOneLight(intersection, scene, ray, sampler);

      LocalRecord sampleScatterRecord = sampleScatter(intersection, ray, sampler);
      if (sampleScatterRecord.f.isZero() || sampleScatterRecord.pdf == 0.f)
          break;
      throughput *= sampleScatterRecord.f * abs(dot(sampleScatterRecord.wi, intersection.normal))
        / sampleScatterRecord.pdf;

      specularBounce = sampleScatterRecord.isDelta;

      ray = Ray(intersection.position + sampleScatterRecord.wi * eps, sampleScatterRecord.wi, eps, 1e10f, .0f,
          intersection.getMedium(sampleScatterRecord.wi));
    }

    float p = russianRoulette(nBounces);
    if (sampler->next1D() > p)
       break;
    throughput /= p;
  }

  return spectrum;
}

// pbrt--v3
Spectrum VolPathIntegrator::uniformSampleOneLight(const Intersection& its, const Scene& scene, const Ray& ray, std::shared_ptr<Sampler> sampler) const {
  // 从场景中采样一个光源
  float pdfLight = .0f;
  auto light = scene.sampleLight(sampler->next1D(), &pdfLight);
  if (light.get() == nullptr || pdfLight == 0)
    return Spectrum(.0f);
  Spectrum direct = estimateDirect(its, ray, light, scene, sampler);
  return direct / pdfLight;
}

// pbrt--v3
Spectrum VolPathIntegrator::estimateDirect(const Intersection& its, const Ray& ray, std::shared_ptr<Light> light, const Scene& scene, std::shared_ptr<Sampler> sampler) const {
  Spectrum ld(.0f);

  // 对光源进行多重重要性采样
  float pdfLight = 0, pdfScatter = 0;
  auto lightSampleResult = light->sample(its, sampler->next2D());
  pdfLight = convertPDF(lightSampleResult, its);
  if (pdfLight > 0 && !lightSampleResult.energy.isZero()) {
    Spectrum f;
    if (its.isSurfaceIts()) {
       f = its.material->computeBSDF(its)->f(-ray.direction, lightSampleResult.direction);
       pdfScatter = squareToCosineHemispherePdf(lightSampleResult.direction);
    }
    else {
       float p = ray.medium->evalPhase(-ray.direction, lightSampleResult.direction);
       f = Spectrum(p);
       pdfScatter = p;
    }

    if (!f.isZero()) {
      Spectrum li = lightSampleResult.energy * evalTransmittance(scene, its, its.position + lightSampleResult.distance * lightSampleResult.direction, sampler);
      if (!li.isZero())
      {
          if (lightSampleResult.isDelta)
            ld += f * li / pdfLight;
          else {
            float weight = powerHeuristic(1, pdfLight, 1, pdfScatter);
            ld += f * li * weight / pdfLight;
          }
      }
    }
  }

  return ld;
}

// 计算光源上某点到交点的光线透射率
Spectrum VolPathIntegrator::evalTransmittance(const Scene &scene, const Intersection &its, Point3f point, std::shared_ptr<Sampler> sampler) const {
  float eps = 1e-4f;
  Vector3f direction = normalize(point - its.position);
  float maxDistance = (point - its.position).length();
  Spectrum tr(1.f);
  Intersection preIntersection = its;
  Intersection intersection = its;

  // 构造从交点到目的点的光线
  Ray ray(its.position + eps * direction, direction, eps, 1e10f, .0f, its.getMedium(direction));
  while (true) {
    auto itsOpt = scene.rayIntersect(ray);
    // 如果从交点到目的点中间没有交点
    if (!itsOpt.has_value()) {
      if (ray.medium.get() != nullptr)
          tr *= ray.medium->evalTransmittance(point, intersection.position, sampler);
      return tr;
    }

    preIntersection = intersection;
    intersection = itsOpt.value();

    // 中间交点的位置超过了maxDistance
    if ((intersection.position - its.position).length() >= maxDistance - eps) {
      if (ray.medium.get() != nullptr)
          tr *= ray.medium->evalTransmittance(point, preIntersection.position, sampler);
      return tr;
    }

    // 有材质的表面，不透光
    if (intersection.isSurfaceIts())
      return Spectrum(.0f);

    if (ray.medium.get() != nullptr)
      tr *= ray.medium->evalTransmittance(intersection.position, preIntersection.position, sampler);

    ray = Ray(intersection.position + direction * eps, direction, eps, 1e10f, .0f, intersection.getMedium(direction));
  }
}

// 采样一条散射光线
LocalRecord VolPathIntegrator::sampleScatter(const Intersection &its, const Ray &ray, std::shared_ptr<Sampler> sampler) const {
  // 分介质交点和表面交点两种情况处理
  if (its.isSurfaceIts()) {
    auto bsdf = its.material->computeBSDF(its);
    auto bsdf_sample_result = bsdf->sample(-ray.direction, sampler->next2D());
    return {bsdf_sample_result.wi, bsdf_sample_result.weight,
            bsdf_sample_result.pdf, bsdf_sample_result.type == BSDFType::Specular};
  }
  else {
    Vector3f wi;
    float pdf = its.getMedium(-ray.direction)->samplePhase(-ray.direction, &wi, sampler->next2D());
    return {wi, pdf, pdf, false};
  }
}

// 计算发光量，如果没有交点，计算环境光
Spectrum VolPathIntegrator::evalEmittance(const Scene &scene, std::optional<Intersection> intersectionOpt, const Ray &ray) const {
  Spectrum emission(.0f);
  if (!intersectionOpt.has_value())
    emission = evalEnvLight(scene, ray);
  else
    if (auto light = intersectionOpt.value().shape->light; light)
      emission += light->evaluateEmission(intersectionOpt.value(), -ray.direction);
  return emission;
}

// 计算环境光
Spectrum VolPathIntegrator::evalEnvLight(const Scene &scene, const Ray &ray) const {
  Spectrum envLight(.0f);
  for (auto light : scene.infiniteLights)
    envLight += light->evaluateEmission(ray);
  return envLight;
}

// 根据介质中的相交信息构造交点
Intersection VolPathIntegrator::fulfillScatterPoint(const Point3f &position, const Vector3f &normal, std::shared_ptr<Medium> medium) const {
  Intersection scatterPoint;
  scatterPoint.position = position;
  scatterPoint.normal = -normal;
  scatterPoint.material = nullptr;
  scatterPoint.mediumInterface = std::make_shared<MediumInterface>(medium);
  return scatterPoint;
}

VolPathIntegrator::VolPathIntegrator(const Json &json) {
  maxDepth = fetchRequired<uint32_t>(json, "maxDepth");
}

REGISTER_CLASS(VolPathIntegrator, "volPath")