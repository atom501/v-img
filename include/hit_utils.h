#pragma once

#include <ray.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <optional>

#include "glm/glm.hpp"
#include "glm/vec3.hpp"

class Material;
class Emitter;

struct HitInfo {
  Material* mat = nullptr;
  const Emitter* obj = nullptr;
  glm::vec3 hit_p;    // point where hit in world coords
  glm::vec3 hit_n_s;  // shading Normal where hit in world coords.
  glm::vec3 hit_n_g;  // geometric Normal where hit in world coords.
                      // Both normals faces are normalized
  glm::vec2 uv;       // texture uv coordinates
};

struct EmitterInfo {
  glm::vec3 wi;  // direction vector from look_from to point on surface
  float pdf;     // solid angle density wrt look_from
  float dist;
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

// ray_dir should be unit vector
inline glm::vec3 project_onto_onb(const ONB& onb, const glm::vec3& ray_dir) {
  return glm::vec3{glm::dot(ray_dir, onb.u), glm::dot(ray_dir, onb.v), glm::dot(ray_dir, onb.w)};
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
  inline std::optional<float> intersect(const Ray& ray, const glm::vec3& ray_inv_dir,
                                        const std::array<bool, 3>& dir_signs) const {
    const float o_x = ray.o[0];
    const float o_y = ray.o[1];
    const float o_z = ray.o[2];

    // x-axis
    const float bmin0 = this->bboxes[dir_signs[0]][0];
    const float bmax0 = this->bboxes[!dir_signs[0]][0];

    const float dmin_x = (bmin0 - o_x) * ray_inv_dir[0];
    const float dmax_x = (bmax0 - o_x) * ray_inv_dir[0];

    // y-axis
    const float bmin1 = this->bboxes[dir_signs[1]][1];
    const float bmax1 = this->bboxes[!dir_signs[1]][1];

    const float dmin_y = (bmin1 - o_y) * ray_inv_dir[1];
    const float dmax_y = (bmax1 - o_y) * ray_inv_dir[1];

    // z-axis
    const float bmin2 = this->bboxes[dir_signs[2]][2];
    const float bmax2 = this->bboxes[!dir_signs[2]][2];

    const float dmin_z = (bmin2 - o_z) * ray_inv_dir[2];
    const float dmax_z = (bmax2 - o_z) * ray_inv_dir[2];

    float tmin = std::max(dmin_x, std::max(dmin_y, std::max(dmin_z, ray.minT)));
    float tmax = std::min(dmax_x, std::min(dmax_y, std::min(dmax_z, ray.maxT)));

    if (tmin <= tmax)
      return tmin;
    else
      return std::nullopt;
  }
};
