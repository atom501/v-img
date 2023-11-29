#pragma once

#include <material/material.h>

#include <glm/vec3.hpp>

class DiffuseLight : public Material {
public:
  glm::vec3 emit;  // The emissive color of the light
  
public:
  DiffuseLight() { emit = glm::vec3(0.5f, 0.5f, 0.5f); };
  ~DiffuseLight(){};

  DiffuseLight(const glm::vec3& emit) : emit(emit) {}

  glm::vec3 emitted(const Ray& ray, const HitInfo& hit) const override {
    // only emit from the normal-facing side
    if (hit.front_face)
      return emit;
    else
      return glm::vec3(0.0f, 0.0f, 0.0f);
  }

  bool is_emissive() const override { return true; }
};
