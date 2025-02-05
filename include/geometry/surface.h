#pragma once

#include <hit_utils.h>
#include <material/material.h>

#include <glm/mat4x4.hpp>
#include <optional>

class Surface {
public:
  Material* mat;

public:
  Surface(Material* in_mat) { mat = in_mat; }
  ~Surface() {};

  virtual std::optional<HitInfo> hit(Ray& r) const = 0;
  virtual AABB bounds() const = 0;
  virtual glm::vec3 get_center() const = 0;

  /*
    input point to sample from (look_from) and 2 random numbers. Return emission from light
    and fill EmitterInfo
  */
  virtual glm::vec3 sample(const glm::vec3& look_from, EmitterInfo& emit_info,
                           pcg32_random_t& pcg_rng) const
      = 0;
  virtual float pdf(const glm::vec3& look_from, const glm::vec3& look_at,
                    const glm::vec3& dir) const
      = 0;
};
