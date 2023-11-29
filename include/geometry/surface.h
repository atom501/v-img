#pragma once

#include <hit_utils.h>
#include <material/material.h>

#include <glm/mat4x4.hpp>

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
};
