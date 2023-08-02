#include "Homogeneous.h"

bool HomogeneousMedium::sampleDistance(MediumSampleResult *mRes, const Ray &ray, std::shared_ptr<Sampler> sampler) const {
    Vector2f sample{sampler->next1D(), sampler->next1D()};

    // 随机选择一个频道并采样一个距离
    int channel = std::min((int)(sample[0] * nSpectrumSamples), nSpectrumSamples - 1);
    float distance = -log(1 - sample[1]) / sigmaT[channel];

    // 采样距离在路径上
    if (distance < ray.tFar) {
        mRes->distance = distance;
        mRes->scatterPoint = ray.at(mRes->distance);
        mRes->sigmaA = sigmaA;
        mRes->sigmaS = sigmaS;
        mRes->tr = evalTransmittance(ray.origin, mRes->scatterPoint, sampler);
        mRes->pdf = .0f;
        for (int i = 0; i < nSpectrumSamples; i++)
            mRes->pdf += sigmaT[i] * exp(-sigmaT[i] * mRes->distance);
        mRes->pdf /= nSpectrumSamples;
        return true;
    }
    // 采样距离比交点距离远
    else {
        mRes->distance = ray.tFar;
        mRes->scatterPoint = ray.at(mRes->distance);
        mRes->sigmaA = sigmaA;
        mRes->sigmaS = sigmaS;
        mRes->tr = evalTransmittance(ray.origin, mRes->scatterPoint, sampler);
        mRes->pdf = .0f;
        for (int i = 0; i < nSpectrumSamples; i++)
            mRes->pdf += exp(-sigmaT[i] * mRes->distance);
        mRes->pdf /= nSpectrumSamples;
        return false;
    }
}

// 计算从 from 到 dest 的光线穿透率
Spectrum HomogeneousMedium::evalTransmittance(Point3f from, Point3f dest, std::shared_ptr<Sampler> sampler) const {
    float distance = (dest - from).length();
    Spectrum tr;
    for (int i = 0; i < nSpectrumSamples; i++)
        tr[i] = exp(-sigmaT[i] * distance);
    return tr;
}

REGISTER_CLASS(HomogeneousMedium, "homogeneous")