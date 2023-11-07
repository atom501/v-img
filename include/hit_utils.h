#pragma once

#include <material/material.h>

#include "glm/glm.hpp"
#include "glm/vec3.hpp"

class Material;

struct HitInfo {
  glm::vec3 hit_p;  // point where hit in world coords
  glm::vec3 hit_n;  // normal where hit in world coords
  glm::vec3 color;
  float t;  // value of t of ray
  Material* mat = nullptr;
};

// ONB for transforming ray directions
struct ONB {
  glm::vec3 u;  // tangent vector
  glm::vec3 v;  // bi-tangent vector
  glm::vec3 w;  // normal vector
};

inline glm::vec3 xform_with_onb(const ONB& onb, const glm::vec3& ray_dir) {
  return (onb.u * ray_dir[0] + onb.v * ray_dir[1] + onb.w * ray_dir[2]);
}

inline ONB init_onb(const glm::vec3& normal_vec) {
  glm::vec3 unit_n = glm::normalize(normal_vec);
  glm::vec3 a = (fabs(unit_n[0]) > 0.9) ? glm::vec3(0, 1, 0) : glm::vec3(1, 0, 0);
  glm::vec3 v = glm::normalize(glm::cross(unit_n, a));
  glm::vec3 u = glm::cross(unit_n, v);

  return {u, v, unit_n};
}
