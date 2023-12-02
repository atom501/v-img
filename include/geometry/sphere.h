#pragma once

#include <geometry/surface.h>
#include <hit_utils.h>
#include <ray.h>

#include <optional>

#include "glm/vec3.hpp"

class Sphere : public Surface {
private:
  glm::vec3 center = glm::vec3(0.0f);
  glm::vec3 color = glm::vec3(1.0f, 0.0f, 0.0f);
  float radius = 1.0f;

public:
  Sphere(const glm::vec3& center, const glm::vec3& color, float r, Material* mat_ptr,
         const glm::mat4x4& xform)
      : color(color), center(center), radius(r), Surface(mat_ptr, xform) {}

  void transform(const glm::mat4& xform) override;
  std::optional<HitInfo> hit(Ray& r) const override;
  AABB bounds() const override;
  glm::vec3 get_center() const override;

  ~Sphere(){};
};
