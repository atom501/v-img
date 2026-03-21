#pragma once

#include <geometry/emitters.h>
#include <geometry/surface.h>
#include <hit_utils.h>
#include <ray.h>

#include "glm/vec3.hpp"

static inline void solveQuadratic(const float& discriminant, const float& a, const float& b_prime,
                                  const float& c, float& x0, float& x1) {
  float sign = (b_prime > 0) ? 1.0f : -1.0f;
  auto q = b_prime + sign * (sqrt(a * discriminant));

  if (discriminant == 0)
    x0 = x1 = c / q;
  else {
    x0 = c / q;
    x1 = q / a;
  }
  if (x0 > x1) std::swap(x0, x1);
}

class Sphere : public Surface, public Emitter {
public:
  glm::vec3 center = glm::vec3(0.0f);
  float radius = 1.0f;
  Material* mat;

  Sphere(const glm::vec3& center, float r, Material* mat_ptr)
      : center(center), radius(r), mat(mat_ptr) {}

  ~Sphere() = default;

  std::optional<ForHitInfo> hit_surface(Ray& r) const override;
  bool hit_check(Ray& r) const override;
  HitInfo hit_info(const Ray& r, const ForHitInfo& pre_calc) const override;

  AABB bounds() const override;
  glm::vec3 get_center() const override;

  std::pair<glm::vec3, EmitterInfo> sample(const glm::vec3& look_from,
                                           pcg32_random_t& pcg_rng) const override;

  float surf_pdf(const glm::vec3& look_from, const glm::vec3& look_at,
                 const glm::vec3& dir) const override;
};
