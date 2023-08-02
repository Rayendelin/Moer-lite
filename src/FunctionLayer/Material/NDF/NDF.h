#pragma once

#include "CoreLayer/Math/Math.h"

class NDF {
public:
  virtual ~NDF() noexcept = default;
  // 法线分布函数
  virtual float getD(const Vector3f &whLocal,
                     const Vector2f &alpha) const noexcept = 0;
  // 几何衰减函数
  virtual float getG(const Vector3f &woLocal, const Vector3f &wiLocal,
                     const Vector2f &alpha) const noexcept = 0;
  virtual float pdf(const Vector3f &woLocal, const Vector3f &whLocal,
                    const Vector2f &alpha) const noexcept = 0;
  virtual Vector3f sampleWh(const Vector3f &woLocal, const Vector2f &alpha,
                            const Vector2f &sample) const noexcept = 0;
};