#pragma once

#include <hit_utils.h>
#include <material/material.h>

#include <glm/mat4x4.hpp>
#include <optional>

class Surface {
public:
  Surface() = default;
  ~Surface() = default;

  virtual std::optional<HitInfo> hit_surface(Ray& r) = 0;
  virtual bool hit_check(Ray& r) = 0;

  virtual AABB bounds() const = 0;
  virtual glm::vec3 get_center() const = 0;
};
