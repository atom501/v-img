#pragma once

#include <material/material.h>

#include <cmath>
#include <cstdint>
#include <optional>

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

// Axis-aligned bounding box
class AABB {
public:
  glm::vec3 box_min;
  glm::vec3 box_min;
  union {
    glm::vec3 bboxes[2];
    struct {
      glm::vec3 box_min;
      glm::vec3 box_max;
    };
  };

public:
  AABB(){};
  AABB(glm::vec3 box_min, glm::vec3 box_max) : box_min(box_min), box_max(box_max) {}

  ~AABB(){};

  uint32_t largest_axis() const {
    auto d = box_max - box_min;
    uint32_t axis = 0;
    if (d[axis] < d[1]) axis = 1;
    if (d[axis] < d[2]) axis = 2;
    return axis;
  }

  void extend(const AABB& box) {
    box_min = glm::min(box_min, box.box_min);
    box_max = glm::max(box_max, box.box_max);
  }

  // half of surface area
  float half_SA() const {
    auto d = box_max - box_min;
    return d[0] * d[1] + d[0] * d[2] + d[1] * d[2];
  }

  std::optional<float> intersect(const Ray& ray) const {
    float tmin = ray.minT, tmax = ray.maxT;
    std::optional<float> t = std::nullopt;

    for (int d = 0; d < 3; ++d) {
      // sign of ray dir
      bool sign = std::signbit(ray.dir[d]);

      const float& bmin = this->bboxes[sign][d];
      const float& bmax = this->bboxes[!sign][d];

      float dmin = (bmin - ray.o[d]) / ray.dir[d];
      float dmax = (bmax - ray.o[d]) / ray.dir[d];

      tmin = std::max(dmin, tmin);
      tmax = std::min(dmax, tmax);
    }

    if (tmin <= tmax) t = tmin;

    return t;
  }
};
