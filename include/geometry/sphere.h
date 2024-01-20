#pragma once

#include <geometry/surface.h>
#include <hit_utils.h>
#include <ray.h>

#include <optional>

#include "glm/vec3.hpp"

class Sphere : public Surface {
private:
  glm::vec3 center = glm::vec3(0.0f);
  float radius = 1.0f;

public:
  Sphere(const glm::vec3& center, float r, Material* mat_ptr)
      : center(center), radius(r), Surface(mat_ptr) {}

  void transform(const glm::mat4& xform) override;
  std::optional<HitInfo> hit(Ray& r) const override;
  AABB bounds() const override;
  glm::vec3 get_center() const override;

  glm::vec3 sample(const glm::vec3& look_from, EmitterInfo& emit_info, float rand1,
                   float rand2) const override;
  float pdf(const glm::vec3& look_from, const glm::vec3& look_at,
            const glm::vec3& dir) const override;

  ~Sphere(){};
};
