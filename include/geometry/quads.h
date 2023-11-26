#pragma once

#include <hit_utils.h>
#include <ray.h>

#include <optional>

#include "glm/vec3.hpp"

class Quad {
public:
  glm::vec3 center = glm::vec3(0.0f);
  glm::vec3 color = glm::vec3(1.0f, 0.0f, 0.0f);
  float radius = 1.0f;
  Material* mat;

public:
  Quad(){};
  Quad(const glm::vec3& center, const glm::vec3& color, float r, Material* mat_ptr)
      : center(center), color(color), radius(r), mat(mat_ptr) {}

  // void transform(const glm::mat4 &xform);
  std::optional<HitInfo> hit(Ray& r) const;

  ~Quad(){};
};