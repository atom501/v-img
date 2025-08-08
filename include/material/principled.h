#pragma once

#include <material/material.h>

#include <algorithm>
#include <numbers>

class Principled : public Material {
private:
  glm::vec3 base_color;
  float specular_transmission;
  float metallic;
  float subsurface;
  float specular;
  float roughness;
  float specular_tint;
  float anisotropic;
  float sheen;
  float sheen_tint;
  float clearcoat;
  float clearcoat_gloss;
  float eta;

public:
  Principled(const glm::vec3& base_color, float specular_transmission, float metallic,
             float subsurface, float specular, float roughness, float specular_tint,
             float anisotropic, float sheen, float sheen_tint, float clearcoat,
             float clearcoat_gloss, float eta)
      : base_color(base_color),
        specular_transmission(specular_transmission),
        metallic(metallic),
        subsurface(subsurface),
        specular(specular),
        roughness(roughness),
        specular_tint(specular_tint),
        anisotropic(anisotropic),
        sheen(sheen),
        sheen_tint(sheen_tint),
        clearcoat(clearcoat),
        clearcoat_gloss(clearcoat_gloss),
        eta(eta) {}
  ~Principled() = default;

  std::optional<ScatterInfo> sample_mat(const glm::vec3& wi, const HitInfo& hit,
                                        pcg32_random_t& pcg_rng) const override;
  glm::vec3 eval(const glm::vec3& wi, const glm::vec3& wo, const HitInfo& hit) const override;
  float pdf(const glm::vec3& wi, const glm::vec3& wo, const HitInfo& hit) const override;

  glm::vec3 eval_div_pdf(const glm::vec3& wi, const glm::vec3& wo,
                         const HitInfo& hit) const override;

  std::pair<glm::vec3, float> eval_pdf_pair(const glm::vec3& wi, const glm::vec3& wo,
                                            const HitInfo& hit) const override;
};
