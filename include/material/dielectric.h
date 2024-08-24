#pragma once

#include <fmt/core.h>
#include <material/material.h>

class Dielectric : public Material {
private:
  float ior;

public:
  Dielectric() { ior = 1; }
  Dielectric(float ior) : ior(ior) {}

  ~Dielectric() {}

  std::optional<ScatterInfo> sample_mat(const glm::vec3& wi, const HitInfo& hit, const float& rand1,
                                        const float& rand2, float rand3) const override;
  glm::vec3 eval(const glm::vec3& wi, const glm::vec3& wo, const HitInfo& hit) const override;
  float pdf(const glm::vec3& wi, const glm::vec3& wo, const HitInfo& hit) const override;
};