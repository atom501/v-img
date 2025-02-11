#pragma once

#include <geometry/surface.h>
#include <hit_utils.h>
#include <ray.h>

#include <optional>

#include "glm/vec3.hpp"

class Quad : public Surface {
private:
  glm::vec3 l_corner = glm::vec3(0.0f);  // lower-left corner of a quad
  glm::vec3 u, v;                        // vectors starting from l_corner
  glm::vec3 normal;
  glm::vec3 w;  // constant for the plane's basis frame
  float D;      // D in eq Ax+By+Cz=D

public:
  Quad(const glm::vec3& l_corner, const glm::vec3& u, const glm::vec3& v, Material* mat_ptr)
      : l_corner(l_corner), u(u), v(v), Surface(mat_ptr) {
    auto n = cross(u, v);
    normal = glm::normalize(n);
    D = glm::dot(normal, l_corner);
    w = n / glm::dot(n, n);
  }

  std::optional<HitInfo> hit(Ray& r) const override;
  AABB bounds() const override;
  glm::vec3 get_center() const override;

  std::pair<glm::vec3, EmitterInfo> sample(const glm::vec3& look_from,
                                           pcg32_random_t& pcg_rng) const override;

  float pdf(const glm::vec3& look_from, const glm::vec3& look_at,
            const glm::vec3& dir) const override;

  ~Quad() {};
};