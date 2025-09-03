#pragma once

#include <fmt/core.h>
#include <material/material.h>

#include <glm/vec3.hpp>
#include <nlohmann/json.hpp>

class DiffuseLight : public Material {
public:
  glm::vec3 emit;  // The emissive color of the light

public:
  DiffuseLight() { emit = glm::vec3(0.5f, 0.5f, 0.5f); };
  ~DiffuseLight() = default;

  DiffuseLight(const glm::vec3& emit) : emit(emit) {}

  DiffuseLight(const nlohmann::json& json_settings) {
    if (json_settings.contains("albedo")) {
      emit[0] = json_settings["albedo"][0];
      emit[1] = json_settings["albedo"][1];
      emit[2] = json_settings["albedo"][2];
    } else {
      emit = glm::vec3(0.5f);
      fmt::println("Lambertian json did not have albedo. Default 0.5 set");
    }
  }

  glm::vec3 emitted(const Ray& ray, const glm::vec3& shading_normal,
                    const glm::vec3& hit_p) const override {
    bool front_face = glm::dot(shading_normal, ray.dir) < 0;
    // only emit from the normal-facing side
    if (front_face)
      return emit;
    else
      return glm::vec3(0.0f, 0.0f, 0.0f);
  }

  bool is_emissive() const override { return true; }

  bool is_delta() const override { return false; }
};
