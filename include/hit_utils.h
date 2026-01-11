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

struct EmitterInfo {
  glm::vec3 wi;  // direction vector from look_from to point on surface
  float pdf;     // pdf of sampling point on light in area measure
  float dist;    // distance to point on light surface
  float G;       // geometry term for point on light
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

inline std::pair<glm::vec3, glm::vec3> get_axis(const glm::vec3& normal_vec) {
  if (normal_vec.z < (-0.9999999f)) {
    return std::make_pair(glm::vec3(0, -1, 0), glm::vec3(-1, 0, 0));
  } else {
    float a = 1.f / (1.f + normal_vec.z);
    float b = -normal_vec.x * normal_vec.y * a;

    return std::make_pair(glm::vec3(1.f - normal_vec.x * normal_vec.x * a, b, -normal_vec.x),
                          glm::vec3(b, 1 - normal_vec.y * normal_vec.y * a, -normal_vec.y));
  }
}

// normal_vec must already be normalized
inline ONB init_onb(const glm::vec3& normal_vec) {
  auto [u, v] = get_axis(normal_vec);
  return ONB{u, v, normal_vec};
}

struct HitInfo {
  Material* mat = nullptr;
  const Emitter* obj = nullptr;
  glm::vec3 hit_p;    // point where hit in world coords
  glm::vec3 hit_n_s;  // shading Normal where hit in world coords.
  glm::vec3 hit_n_g;  // geometric Normal where hit in world coords.
                      // Both normals faces are normalized
  glm::vec2 uv;       // texture uv coordinates
  ONB n_frame;
  float primitive_area;
  float tex_coord_area;
  float mean_curvature;
};

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

  void extend(const glm::vec3& point) {
    bboxes[0] = glm::min(bboxes[0], point);
    bboxes[1] = glm::max(bboxes[1], point);
  }

  // half of surface area
  float half_SA() const {
    auto d = bboxes[1] - bboxes[0];
    return d[0] * d[1] + d[0] * d[2] + d[1] * d[2];
  }

  float surface_area() const {
    auto d = bboxes[1] - bboxes[0];
    return 2.f * (d[0] * d[1] + d[0] * d[2] + d[1] * d[2]);
  }
};

// ray-aabb slab test. result is returned in t_val variable
inline float slab_intersect_aabb_array(const Ray& ray, const glm::vec3& ray_inv_dir,
                                       const std::array<float, 3>& bb_min,
                                       const std::array<float, 3>& bb_max) {
  glm::vec3 min_vec3 = glm::vec3(bb_min[0], bb_min[1], bb_min[2]);
  glm::vec3 max_vec3 = glm::vec3(bb_max[0], bb_max[1], bb_max[2]);

  glm::vec3 tLower = (min_vec3 - ray.o) * ray_inv_dir;
  glm::vec3 tUpper = (max_vec3 - ray.o) * ray_inv_dir;
  glm::vec4 tMins = glm::vec4(glm::min(tLower, tUpper), ray.minT);
  glm::vec4 tMaxes = glm::vec4(glm::max(tLower, tUpper), ray.maxT);
  float tBoxMin = std::max(tMins.x, std::max(tMins.y, std::max(tMins.z, tMins.w)));
  float tBoxMax = std::min(tMaxes.x, std::min(tMaxes.y, std::min(tMaxes.z, tMaxes.w)));

  if (tBoxMin <= tBoxMax)
    return tBoxMin;
  else
    return std::numeric_limits<float>::infinity();
}
