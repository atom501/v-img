#pragma once

#include <hit_utils.h>
#include <material/material.h>

#include <glm/mat4x4.hpp>
#include <optional>

class Surface {
public:
  Material* mat;
  // keep these two incase I need to store them for textures.
  // right now will transform the object geometry at start and to ray-object intersection completely
  // in world space
  glm::mat4x4 obj_to_world;
  glm::mat4x4 world_to_obj;

public:
  Surface(Material* in_mat, const glm::mat4x4& in_obj_to_world) {
    mat = in_mat;
    obj_to_world = in_obj_to_world;
    world_to_obj = glm::inverse(obj_to_world);
  }
  ~Surface(){};

  virtual std::optional<HitInfo> hit(Ray& r) const = 0;
  virtual void transform(const glm::mat4& xform) = 0;
  virtual AABB bounds() const = 0;
  virtual glm::vec3 get_center() const = 0;

  /*
    input point to sample from (look_from) and 2 random numbers. Return emission from light
    and fill EmitterInfo
  */
  virtual glm::vec3 sample(const glm::vec3& look_from, EmitterInfo& emit_info, float rand1,
                           float rand2) const
      = 0;
  virtual float pdf(const glm::vec3& look_from, const glm::vec3& look_at,
                    const glm::vec3& dir) const
      = 0;
};
