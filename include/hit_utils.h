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
  glm::vec2 uv;     // texture uv coordinates
  bool front_face;  // tell if hit front face or not. normal flipped if false
};

struct EmitterInfo {
  glm::vec3 wi;  // direction vector from look_from to point on surface
  float pdf;     // solid angle density wrt look_from
  float dist;
  const Surface* obj = nullptr;  // ptr of surface hit
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
  AABB() = default;
  AABB(const glm::vec3& box_min, const glm::vec3& box_max) {
    bboxes[0] = box_min;
    bboxes[1] = box_max;
  }

  ~AABB() = default;

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

  /*
   * source is https://tavianator.com/2022/ray_box_boundary.html
   * loop is manually unrolled and ray inverse calculated in bvh hit call
   */
  std::optional<float> intersect(const Ray& ray, const glm::vec3& ray_inv_dir) const {
    float tmin = ray.minT, tmax = ray.maxT;
    std::optional<float> t = std::nullopt;
    bool sign;
    float dmin, dmax;

    // x-axis
    sign = std::signbit(ray.dir[0]);

    const float bmin0 = this->bboxes[sign][0];
    const float bmax0 = this->bboxes[!sign][0];

    dmin = (bmin0 - ray.o[0]) * ray_inv_dir[0];
    dmax = (bmax0 - ray.o[0]) * ray_inv_dir[0];

    tmin = std::max(dmin, tmin);
    tmax = std::min(dmax, tmax);

    // y-axis
    sign = std::signbit(ray.dir[1]);

    const float bmin1 = this->bboxes[sign][1];
    const float bmax1 = this->bboxes[!sign][1];

    dmin = (bmin1 - ray.o[1]) * ray_inv_dir[1];
    dmax = (bmax1 - ray.o[1]) * ray_inv_dir[1];

    tmin = std::max(dmin, tmin);
    tmax = std::min(dmax, tmax);

    // z-axis
    sign = std::signbit(ray.dir[2]);

    const float bmin2 = this->bboxes[sign][2];
    const float bmax2 = this->bboxes[!sign][2];

    dmin = (bmin2 - ray.o[2]) * ray_inv_dir[2];
    dmax = (bmax2 - ray.o[2]) * ray_inv_dir[2];

    tmin = std::max(dmin, tmin);
    tmax = std::min(dmax, tmax);

    if (tmin <= tmax) t = tmin;

    return t;
  }
};
