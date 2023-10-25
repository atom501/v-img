#pragma once

#include <hit_utils.h>
#include <ray.h>

#include <optional>

#include "glm/vec3.hpp"

class Sphere {
public:
  glm::vec3 center = glm::vec3(0.0f);
  glm::vec3 color = glm::vec3(1.0f, 0.0f, 0.0f);
  float radius = 1.0f;

public:
  Sphere(){};
  Sphere(const glm::vec3& center, const glm::vec3& color, float r)
      : center(center), color(color), radius(r) {}

  // void transform(const glm::mat4 &xform);
  std::optional<HitInfo> hit(const Ray& r) const;

  ~Sphere(){};
};
