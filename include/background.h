#pragma once
#include <geometry/emitters.h>
#include <ray.h>

#include <algorithm>
#include <utility>
#include <vector>

#include "glm/vec3.hpp"

class Background {
public:
  Background() = default;
  ~Background() = default;

  virtual glm::vec3 background_emit(const Ray& in_ray) const = 0;
};

class ConstBackground : public Background, public Emitter {
private:
  glm::vec3 col;

public:
  ConstBackground(const glm::vec3& const_col) : col(const_col) {}
  ~ConstBackground() = default;

  glm::vec3 background_emit(const Ray& in_ray) const override { return col; }

  std::pair<glm::vec3, EmitterInfo> sample(const glm::vec3& look_from,
                                           pcg32_random_t& pcg_rng) const {
    float r1 = rand_float(pcg_rng);
    float r2 = rand_float(pcg_rng);
    // sample uniform sphere
    glm::vec3 wi = sample_sphere(r1, r2);

    constexpr float pdf = 1.f / (4 * M_PI);

    constexpr float dist = std::numeric_limits<float>::infinity();

    return std::make_pair(ConstBackground::col, EmitterInfo{wi, pdf, dist});
  }

  float pdf(const glm::vec3& look_from, const glm::vec3& look_at, const glm::vec3& dir) const {
    return 1.f / (4 * M_PI);
  }
};

// image loaded in from a file
class EnvMap : public Background, public Emitter {
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
  glm::vec3 background_emit(const Ray& in_ray) const override {
    // transform from ray dir world space to obj/env space
    glm::vec3 dir = glm::vec3(world_to_env * glm::vec4(in_ray.dir, 0.0f));

    // get uv coordinates. Envmap assumes latitude-longitude format. same as in mitsuba
    dir = normalize(dir);
    float u = (1.f + atan2(dir.x, dir.z) * M_1_PI) * 0.5f;
    float v = acos(dir.y) * M_1_PI;

    float pixel_u = (1.0f - u) * width;
    float pixel_v = v * height;

    // get pixel value using sampling
    int curr_x = std::clamp(static_cast<int>(pixel_u), 0, static_cast<int>(width) - 1);
    int curr_y = std::clamp(static_cast<int>(pixel_v), 0, static_cast<int>(height) - 1);

    int next_x = std::clamp(curr_x + 1, 0, static_cast<int>(width) - 1);
    int next_y = std::clamp(curr_y + 1, 0, static_cast<int>(height) - 1);

    float x_fraction = pixel_u - curr_x;
    float y_fraction = pixel_v - curr_y;

    glm::vec3 x0 = image[curr_x + curr_y * width];
    glm::vec3 x1 = image[next_x + curr_y * width];

    glm::vec3 a = glm::mix(x0, x1, glm::vec3(x_fraction));

    glm::vec3 y0 = image[curr_x + next_y * width];
    glm::vec3 y1 = image[next_x + next_y * width];

    glm::vec3 b = glm::mix(y0, y1, glm::vec3(x_fraction));

    return glm::mix(a, b, glm::vec3(y_fraction));
  }

  std::pair<glm::vec3, EmitterInfo> sample(const glm::vec3& look_from,
                                           pcg32_random_t& pcg_rng) const override {
    return std::make_pair(glm::vec3(0.f), EmitterInfo{glm::vec3(0.f), 0.f, 0.f});
  }
  float pdf(const glm::vec3& look_from, const glm::vec3& look_at,
            const glm::vec3& dir) const override {
    return 0.f;
  }
};
