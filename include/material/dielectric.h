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

  ~Dielectric() {}

  std::optional<ScatterInfo> sample_mat(const glm::vec3& wi, const HitInfo& hit, const float& rand1,
                                        const float& rand2, float rand3) const override;
  glm::vec3 eval(const glm::vec3& wi, const glm::vec3& wo, const HitInfo& hit) const override;
  float pdf(const glm::vec3& wi, const glm::vec3& wo, const HitInfo& hit) const override;
};