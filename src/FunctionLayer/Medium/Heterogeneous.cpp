#include "Heterogeneous.h"

bool GridDensityMedium::sampleDistance(MediumSampleResult *mRes, const Ray &ray, std::shared_ptr<Sampler> sampler) const {
    // 将光线转换到局部坐标系
    Ray localRay = mediumToWorld.inverseRayNew(ray);
    // 判断光线是否与网格相交
    AABB box(Point3f(0, 0, 0), Point3f(1, 1, 1));
    float tMin, tMax;
    if (box.RayIntersect(localRay, &tMin, &tMax)) {
        float t = tMin;
        while (true) {
            t -= log(1 - sampler->next1D()) * invMaxDensity / sigmaT[0];
            if (t >= tMax)
                break;
            Point3f p = localRay.at(t);
            if (evalDensity(p) * invMaxDensity > sampler->next1D()) {
                mRes->distance = t;
                mRes->scatterPoint = mediumToWorld.toWorld(p);
                mRes->sigmaA = sigmaA;
                mRes->sigmaS = sigmaS;
                mRes->tr = 1 / sigmaT[0];
                mRes->pdf = 1.f;
                return true;
            }
        }
    }
    mRes->distance = ray.tFar;
    mRes->scatterPoint = ray.at(mRes->distance);
    mRes->sigmaA = sigmaA;
    mRes->sigmaS = sigmaS;
    mRes->tr = Spectrum(1.f);
    mRes->pdf = 1.f;
    return false;
}

// 计算从 from 到 dest 的光线穿透率
Spectrum GridDensityMedium::evalTransmittance(Point3f from, Point3f dest, std::shared_ptr<Sampler> sampler) const {
    Ray ray(from, dest);
    Ray localRay = mediumToWorld.inverseRayNew(ray);
    // 判断光线是否与网格相交
    AABB box(Point3f(0, 0, 0), Point3f(1, 1, 1));
    float tMin, tMax;
    if (!box.RayIntersect(localRay, &tMin, &tMax))
        return Spectrum(1.f);

    float tr = 1, t = tMin;
    while (true) {
        t -= log(1 - sampler->next1D()) * invMaxDensity / sigmaT[0];
        if (t >= tMax)
            break;
        float density = evalDensity(localRay.at(t));
        tr *= 1 - std::max((float)0, density * invMaxDensity);
        if (tr < .1f) {
            float q = std::max(.05f, 1 - tr);
            if (sampler->next1D() < q)
                return 0;
            tr /= 1 - q;
        }
    }
    return Spectrum(tr);
}

// 假设网格样本位于规范域[0,1]^3上
float GridDensityMedium::evalDensity(Point3f point) const {
    Point3f pSamples(point[0] * nx - .5f, point[1] * ny - .5f, point[2] * nz - .5f);
    int x = (int)floor(pSamples[0]), y = (int)floor(pSamples[1]), z = (int)floor(pSamples[2]);
    Vector3f d = pSamples - Point3f(x, y, z);

    float d00 = lerp(d[0], D(x, y, z), D(x + 1, y, z));
    float d10 = lerp(d[0], D(x, y + 1, z), D(x + 1, y + 1, z));
    float d01 = lerp(d[0], D(x, y, z + 1), D(x + 1, y, z + 1));
    float d11 = lerp(d[0], D(x, y + 1, z + 1), D(x + 1, y + 1, z + 1));
    float d0 = lerp(d[1], d00, d10);
    float d1 = lerp(d[1], d01, d11);
    return lerp(d[2], d0, d1);
}

REGISTER_CLASS(GridDensityMedium, "gridDensity");