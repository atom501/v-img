#pragma once

#include <material/material.h>
#include <rng/sampling.h>

#include <algorithm>
#include <cmath>

class Lambertian : public Material {
private:
  glm::vec3 albedo;

public:
  Lambertian() { albedo = glm::vec3(0.0f); };

  Lambertian(const glm::vec3& albedo) : albedo(albedo) {}

  ~Lambertian(){};

  std::optional<ScatterInfo> sample_mat(const glm::vec3& wi, const HitInfo& hit, const float& rand1,
                                        const float& rand2, float rand3) const override;
  glm::vec3 eval(const glm::vec3& wi, const glm::vec3& wo, const HitInfo& hit) const override;
  float pdf(const glm::vec3& wi, const glm::vec3& wo, const HitInfo& hit) const override;
};
