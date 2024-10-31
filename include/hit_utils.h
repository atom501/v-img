#pragma once

#include <ray.h>

#include <cmath>
#include <cstdint>
#include <optional>

#include "glm/glm.hpp"
#include "glm/vec3.hpp"

class Material;
class Surface;

struct HitInfo {
  Material* mat = nullptr;
  const Surface* obj = nullptr;
  glm::vec3 hit_p;  // point where hit in world coords
  glm::vec3 hit_n;  // Normal where hit in world coords.
                    // Always faces towards the incoming ray, always normalized
  bool front_face;  // tell if hit front face or not. normal flipped if false
};

struct EmitterInfo {
  glm::vec3 wi;  // direction vector from look_from to point on surface
  float pdf;     // solid angle density wrt look_from
  HitInfo hit;   // point on surface information
};

// ONB for transforming ray directions. All are unit vectors
struct ONB {
  glm::vec3 u;  // tangent vector
  glm::vec3 v;  // bi-tangent vector
  glm::vec3 w;  // normal vector
};

// ray_dir should be unit vector
inline glm::vec3 xform_with_onb(const ONB& onb, const glm::vec3& ray_dir) {
  return (onb.u * ray_dir[0] + onb.v * ray_dir[1] + onb.w * ray_dir[2]);
}

// normal_vec must already be normalized
inline ONB init_onb(const glm::vec3& normal_vec) {
  glm::vec3 unit_n = normal_vec;
  glm::vec3 a = (fabs(unit_n[0]) > 0.9) ? glm::vec3(0, 1, 0) : glm::vec3(1, 0, 0);
  glm::vec3 v = glm::normalize(glm::cross(unit_n, a));
  glm::vec3 u = glm::cross(unit_n, v);  // already a unit vector as unit_n and v are perpendicular

  return ONB{u, v, unit_n};
}

// Axis-aligned bounding box
class AABB {
public:
  glm::vec3 bboxes[2];  // 0 index min bounding box. 1 index max bounding box

public:
  AABB() {};
  AABB(const glm::vec3& box_min, const glm::vec3& box_max) {
    bboxes[0] = box_min;
    bboxes[1] = box_max;
  }

  ~AABB() {};

  uint32_t largest_axis() const {
    auto d = bboxes[1] - bboxes[0];
    uint32_t axis = 0;
    if (d[axis] < d[1]) axis = 1;
    if (d[axis] < d[2]) axis = 2;
    return axis;
  }

  void extend(const AABB& box) {
    bboxes[0] = glm::min(bboxes[0], box.bboxes[0]);
    bboxes[1] = glm::max(bboxes[1], box.bboxes[1]);
  }

  // half of surface area
  float half_SA() const {
    auto d = bboxes[1] - bboxes[0];
    return d[0] * d[1] + d[0] * d[2] + d[1] * d[2];
  }

  std::optional<float> intersect(const Ray& ray) const {
    float tmin = ray.minT, tmax = ray.maxT;
    std::optional<float> t = std::nullopt;
    float inv_dir = 0;

    for (int d = 0; d < 3; ++d) {
      // sign of ray dir
      bool sign = std::signbit(ray.dir[d]);
      inv_dir = 1 / ray.dir[d];

      const float& bmin = this->bboxes[sign][d];
      const float& bmax = this->bboxes[!sign][d];

      float dmin = (bmin - ray.o[d]) * inv_dir;
      float dmax = (bmax - ray.o[d]) * inv_dir;

      tmin = std::max(dmin, tmin);
      tmax = std::min(dmax, tmax);
    }

    if (tmin <= tmax) t = tmin;

    return t;
  }
};
