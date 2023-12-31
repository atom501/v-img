#pragma once

#include <fmt/core.h>
#include <material/material.h>
#include <rng/sampling.h>

#include <algorithm>
#include <cmath>
#include <nlohmann/json.hpp>

class Lambertian : public Material {
private:
  glm::vec3 albedo;

public:
  Lambertian() { albedo = glm::vec3(0.0f); };

  Lambertian(const glm::vec3& albedo) : albedo(albedo) {}

  Lambertian(const nlohmann::json& json_settings) {
    if (json_settings.contains("albedo")) {
      albedo[0] = json_settings["albedo"][0];
      albedo[1] = json_settings["albedo"][1];
      albedo[2] = json_settings["albedo"][2];
    } else {
      albedo = glm::vec3(0.0f);
      fmt::println("Lambertian json did not have albedo. Default 0 set");
    }
  }

  ~Lambertian(){};

  std::optional<ScatterInfo> sample_mat(const glm::vec3& wi, const HitInfo& hit, const float& rand1,
                                        const float& rand2, float rand3) const override;
  glm::vec3 eval(const glm::vec3& wi, const glm::vec3& wo, const HitInfo& hit) const override;
  float pdf(const glm::vec3& wi, const glm::vec3& wo, const HitInfo& hit) const override;
};
