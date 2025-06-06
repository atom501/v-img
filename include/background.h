#pragma once
#include <ray.h>

#include <vector>

#include "glm/vec3.hpp"

class Background {
public:
  Background() = default;
  ~Background() = default;

  virtual glm::vec3 background_emit(const Ray& in_ray) const = 0;
};

class ConstBackground : public Background {
private:
  glm::vec3 col;

public:
  ConstBackground(const glm::vec3& const_col) : col(const_col) {}
  ~ConstBackground();

  glm::vec3 background_emit(const Ray& in_ray) const override { return col; }
};

// image loaded in from a file
class EnvMap : public Background {
private:
  uint32_t width;
  uint32_t height;
  std::vector<glm::vec3> image;  // flat image. size width * height
  glm::mat4 world_to_env;

public:
  EnvMap(uint32_t width, uint32_t height, std::vector<glm::vec3>& image,
         const glm::mat4& world_to_env)
      : width(width), height(height), image(image), world_to_env(world_to_env) {}
  ~EnvMap() = default;

  // transform ray from world to image space. Then get emitted color
  glm::vec3 background_emit(const Ray& in_ray) const override { return glm::vec3(0); }
};
