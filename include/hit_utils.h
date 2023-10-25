#pragma once

#include "glm/vec3.hpp"

struct HitInfo {
  glm::vec3 hit_p;  // point where hit in world coords
  glm::vec3 hit_n;  // normal where hit in world coords
  glm::vec3 color;
  float t;  // value of t of ray
};
