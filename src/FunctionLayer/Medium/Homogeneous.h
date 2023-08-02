#pragma once
#include "Medium.h"

class HomogeneousMedium: public Medium {
public:
    HomogeneousMedium(Spectrum sigmaA, Spectrum sigmaS, std::shared_ptr<PhaseFunction> phase):
        Medium(phase), sigmaA(sigmaA), sigmaS(sigmaS), sigmaT(sigmaA+sigmaS) {}

    HomogeneousMedium(const Json& json): Medium(json) {
        sigmaA = fetchRequired<Spectrum>(json, "sigmaA");
        sigmaS = fetchRequired<Spectrum>(json, "sigmaS");
        sigmaT = sigmaA + sigmaS;
    }

    virtual bool sampleDistance(MediumSampleResult *mRes, const Ray &ray
        , std::shared_ptr<Sampler> sampler) const override;
    virtual Spectrum evalTransmittance(Point3f from, Point3f dest, std::shared_ptr<Sampler> sampler) const override;

private:
    Spectrum sigmaA, sigmaS, sigmaT;
};