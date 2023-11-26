#pragma once

#include <material/material.h>

#include <glm/vec3.hpp>

class DiffuseLight : public Material {
public:
  DiffuseLight() { emit = glm::vec3(0.5f, 0.5f, 0.5f); };
  ~DiffuseLight(){};

  DiffuseLight(const glm::vec3& emit) : emit(emit) {}

  glm::vec3 emitted(const Ray& ray, const HitInfo& hit) const override {
    // only emit from the normal-facing side
    if (glm::dot(ray.d, hit.hit_n) > 0)
      return glm::vec3(0.0f, 0.0f, 0.0f);
    else
      return emit;
  }

  bool is_emissive() const override { return true; }

public:
  glm::vec3 emit;  // The emissive color of the light
};
