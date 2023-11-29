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
  glm::vec3 color = glm::vec3(1.0f, 0.0f, 0.0f);
  double D;  // D in eq Ax+By+Cz=D

public:
  Quad(const glm::vec3& color, const glm::vec3& l_corner, const glm::vec3& u, const glm::vec3& v,
       Material* mat_ptr, const glm::mat4x4& xform)
      : color(color), l_corner(l_corner), u(u), v(v), Surface(mat_ptr, xform) {
    auto n = cross(u, v);
    normal = glm::normalize(n);
    D = glm::dot(normal, l_corner);
  }

  void transform(const glm::mat4& xform) override;
  std::optional<HitInfo> hit(Ray& r) const override;
  AABB bounds() const override;

  ~Quad(){};
};