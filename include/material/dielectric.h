#pragma once

#include <fmt/core.h>
#include <material/material.h>

#include <nlohmann/json.hpp>

class Dielectric : public Material {
private:
  float ior;

public:
  Dielectric() { ior = 1; }
  Dielectric(float ior) : ior(ior) {}

  Dielectric(const nlohmann::json& json_settings) {
    if (json_settings.contains("ior")) {
      ior = json_settings["ior"];
    } else {
      ior = 1.5f;
      fmt::println("Dielectric json did not have ior. Default 1.5 set");
    }
  }

  ~Dielectric() = default;

  std::optional<ScatterInfo> sample_mat(const glm::vec3& wi, const HitInfo& hit,
                                        pcg32_random_t& pcg_rng, bool regularize) const override;
  glm::vec3 eval(const glm::vec3& wi, const glm::vec3& wo, const HitInfo& hit,
                 const RayCone& cone) const override;
  float pdf(const glm::vec3& wi, const glm::vec3& wo, const HitInfo& hit) const override;
  glm::vec3 eval_div_pdf(const glm::vec3& wi, const glm::vec3& wo, const HitInfo& hit,
                         const RayCone& cone, bool regularize) const override;
  bool is_delta() const override { return true; }
};