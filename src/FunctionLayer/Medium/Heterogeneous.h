#pragma once
#include <CoreLayer/Math/Transform.h>
#include "Medium.h"

// 在两个提供的数之间进行线性插值
inline float lerp(float t, float v1, float v2) {
    return (1 - t) * v1 + t * v2;
}

class GridDensityMedium: public Medium {
public:
    GridDensityMedium(Spectrum& sigmaA, Spectrum& sigmaS, std::shared_ptr<PhaseFunction> phase,
        int nx, int ny, int nz, Transform& mediumToWorld, float* d):
        Medium(phase), sigmaA(sigmaA), sigmaS(sigmaS), sigmaT(sigmaA+sigmaS), 
        nx(nx), ny(ny), nz(nz), mediumToWorld(mediumToWorld), density(new float[nx*ny*nz])
        {
            memcpy(density, d, sizeof(float) * nx * ny * nz);
            float maxDensity = .0f;
            for (int i = 0; i < nx * ny * nz; i++)
                maxDensity = std::max(maxDensity, density[i]);
            invMaxDensity = 1 / maxDensity;
        }

    GridDensityMedium(const Json& json): Medium(json) {
        sigmaA = fetchRequired<Spectrum>(json, "sigmaA");
        sigmaS = fetchRequired<Spectrum>(json, "sigmaS");
        sigmaT = sigmaA + sigmaS;
        nx = fetchRequired<int>(json, "nx");
        ny = fetchRequired<int>(json, "ny");
        nz = fetchRequired<int>(json, "nz");
        
        if (json.contains("transform")) {
            Vector3f translate = fetchOptional(json["transform"], "translate", Vector3f{0.f, 0.f, 0.f});
            Vector3f scale = fetchOptional(json["transform"], "scale", Vector3f{1.f, 1.f, 1.f});

            Matrix4f translateMat = Transform::translation(translate);
            Matrix4f scaleMat = Transform::scalation(scale);
            Matrix4f rotateMat = Matrix4f::identity();
            if (json["transform"].contains("rotate")) {
                Vector3f axis = fetchRequired<Vector3f>(json["transform"]["rotate"], "axis");
                float radian = fetchRequired<float>(json["transform"]["rotate"], "radian");
                rotateMat = Transform::rotation(axis, radian);
            }
            mediumToWorld = Transform{translateMat, rotateMat, scaleMat};
        }

        from_json(json["density"], density, nx * ny * nz);

        float maxDensity = 0.f;
        for (int i = 0; i < nx * ny * nz; i++)
            maxDensity = std::max(maxDensity, density[i]);
        invMaxDensity = 1 / maxDensity;
    }

    virtual bool sampleDistance(MediumSampleResult *mRes, const Ray &ray, std::shared_ptr<Sampler> sampler) const override;
    virtual Spectrum evalTransmittance(Point3f from, Point3f dest, std::shared_ptr<Sampler> sampler) const override;

    float D(int x, int y, int z) const {
        if (x >= 0 && x < nx && y >= 0 && y < ny && z >= 0 && z < nz)
                return density[(z * ny + y) * nx + x];
        return 0;
    }

    float evalDensity(Point3f point) const;

private:
    Spectrum sigmaA, sigmaS, sigmaT;
    int nx, ny, nz;
    Transform mediumToWorld;
    float* density;
    float invMaxDensity;
};