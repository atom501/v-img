#pragma once

#include <fmt/core.h>
#include <material/material.h>
#include <rng/sampling.h>
#include <texture.h>

#include <algorithm>
#include <cmath>
#include <nlohmann/json.hpp>

class Lambertian : public Material {
private:
  Texture* tex;

public:
  Lambertian() { tex = nullptr; }

  Lambertian(Texture* tex) : tex(tex) {}

  ~Lambertian() = default;

  std::optional<ScatterInfo> sample_mat(const glm::vec3& wi, const HitInfo& hit,
                                        pcg32_random_t& pcg_rng, bool regularize) const override;
  glm::vec3 eval(const glm::vec3& wi, const glm::vec3& wo, const HitInfo& hit,
                 const RayCone& cone) const override;
  float pdf(const glm::vec3& wi, const glm::vec3& wo, const HitInfo& hit) const override;

  glm::vec3 eval_div_pdf(const glm::vec3& wi, const glm::vec3& wo, const HitInfo& hit,
                         const RayCone& cone, bool regularize) const override;

  std::pair<glm::vec3, float> eval_pdf_pair(const glm::vec3& wi, const glm::vec3& wo,
                                            const HitInfo& hit, const RayCone& cone,
                                            bool regularize) const override;

  bool is_delta() const override { return false; }
};
