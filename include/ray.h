#pragma once

#include <limits>

#include "glm/mat4x4.hpp"
#include "glm/vec3.hpp"

class Ray {
public:
  glm::vec3 dir = glm::vec3(1.0f);
  glm::vec3 o = glm::vec3(0.0f);

  float minT = 0.0001f;
  float maxT = std::numeric_limits<float>::infinity();

public:
  Ray(glm::vec3& o, glm::vec3& d) : dir(d), o(o) {}
  Ray() {}
  ~Ray() {}

  glm::vec3 at(float t) { return o + t * dir; }

  // transform the current ray
  void xform_ray(const glm::mat4& xform) {
    dir = glm::vec3(xform * glm::vec4(dir, 0.0f));
    glm::vec4 temp_o = xform * glm::vec4(o, 1.0f);
    temp_o /= temp_o[3];
    o = glm::vec3(temp_o);
  }
};
