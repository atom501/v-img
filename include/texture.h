#pragma once

#include "glm/vec2.hpp"
#include "glm/vec3.hpp"

class Texture {
public:
  Texture() = default;
  ~Texture() = default;

  virtual glm::vec3 col_at_uv(const glm::vec2& uv) const = 0;
};

class ConstColor : public Texture {
private:
  glm::vec3 albedo;

public:
  ConstColor(const glm::vec3& albedo) : albedo(albedo) {}
  ~ConstColor() = default;

  glm::vec3 col_at_uv(const glm::vec2& uv) const override { return albedo; }
};
