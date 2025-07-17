#pragma once
#include <geometry/emitters.h>
#include <ray.h>

#include <algorithm>
#include <numbers>
#include <utility>
#include <vector>

#include "glm/vec3.hpp"

class Background {
public:
  Background() = default;
  ~Background() = default;

  virtual glm::vec3 background_emit(const Ray& in_ray) const = 0;
  virtual bool is_emissive() const = 0;
  virtual float background_pdf(const glm::vec3& dir) const = 0;
};

class ConstBackground : public Background, public Emitter {
private:
  glm::vec3 col;

public:
  ConstBackground(const glm::vec3& const_col) : col(const_col) {}
  ~ConstBackground() = default;

  glm::vec3 background_emit(const Ray& in_ray) const override { return col; }

  std::pair<glm::vec3, EmitterInfo> sample(const glm::vec3& look_from,
                                           pcg32_random_t& pcg_rng) const override {
    float r1 = rand_float(pcg_rng);
    float r2 = rand_float(pcg_rng);
    // sample uniform sphere
    glm::vec3 wi = sample_sphere(r1, r2);

    constexpr float pdf = 1.f / (4 * std::numbers::pi);

    constexpr float dist = std::numeric_limits<float>::infinity();

    return std::make_pair(ConstBackground::col, EmitterInfo{wi, pdf, dist});
  }

  float background_pdf(const glm::vec3& dir) const override { return 1.f / (4 * std::numbers::pi); }

  bool is_emissive() const override {
    if (col == glm::vec3(0.f))
      return false;
    else
      return true;
  }

  bool is_background() const override { return true; }

  float pdf(const glm::vec3& look_from, const glm::vec3& look_at,
            const glm::vec3& dir) const override {
    return 0.f;
  }
};

// image loaded in from a file
class EnvMap : public Background, public Emitter {
private:
  uint32_t width;
  uint32_t height;
  std::vector<glm::vec3> image;  // flat image. size width * height. For exr image loaded using
                                 // tinyexr, top left corner is 0,0 index
  glm::mat4 world_to_env;
  glm::mat4 env_to_world;
  ArraySampling2D image_sampling;
  float radiance_scale;

public:
  EnvMap(uint32_t width, uint32_t height, std::vector<glm::vec3>& image,
         const glm::mat4& world_to_env, const glm::mat4& env_to_world, float radiance_scale)
      : width(width),
        height(height),
        image(image),
        world_to_env(world_to_env),
        env_to_world(env_to_world),
        radiance_scale(radiance_scale) {
    image_sampling = ArraySampling2D(image, width, height);
  }
  ~EnvMap() = default;

  // transform ray from world to image space. Then get emitted color
  glm::vec3 background_emit(const Ray& in_ray) const override {
    // transform from ray dir world space to obj/env space
    glm::vec3 dir = glm::vec3(world_to_env * glm::vec4(in_ray.dir, 0.0f));

    // get uv coordinates. Envmap assumes latitude-longitude format. same as in mitsuba
    dir = glm::normalize(dir);
    float u = (1.f + std::atan2(-dir.x, dir.z) * std::numbers::inv_pi) * 0.5f;
    float v = std::acos(dir.y) * std::numbers::inv_pi;

    return col_from_uv(u, v) * radiance_scale;
  }

  glm::vec3 col_from_uv(float u, float v) const {
    float pixel_u = u * width;
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
    float r1 = rand_float(pcg_rng);
    float r2 = rand_float(pcg_rng);

    // sample image
    auto [u_env, v_env, choose_sample_pdf] = image_sampling.sample(r1, r2);

    // convert u, v to a direction

    float elevation = v_env * std::numbers::pi;
    float y = std::cos(v_env * std::numbers::pi);

    const float azimuth = u_env * 2.f * std::numbers::pi;
    float x = std::sin(azimuth) * std::sin(elevation);
    float z = -1 * std::cos(azimuth) * std::sin(elevation);

    glm::vec3 wi(x, y, z);
    wi = glm::vec3(env_to_world * glm::vec4(wi, 0.0f));
    wi = glm::normalize(wi);

    constexpr float dist = std::numeric_limits<float>::infinity();

    float sin_elevation = std::sin(elevation);
    float pdf = (choose_sample_pdf * width * height)
                / (2.f * std::numbers::pi * std::numbers::pi * sin_elevation);

    return std::make_pair(col_from_uv(u_env, v_env) * radiance_scale, EmitterInfo{wi, pdf, dist});
  }

  float background_pdf(const glm::vec3& in_dir) const override {
    // transform from ray dir world space to obj/env space
    glm::vec3 dir = glm::vec3(world_to_env * glm::vec4(in_dir, 0.0f));

    // get uv coordinates. Envmap assumes latitude-longitude format. same as in mitsuba
    dir = glm::normalize(dir);
    float u = (1.f + std::atan2(-dir.x, dir.z) * std::numbers::inv_pi) * 0.5f;
    float v = std::acos(dir.y) * std::numbers::inv_pi;

    // use u, v to get pdf
    int pixel_u = u * width;
    int pixel_v = v * height;

    // get pixel value using sampling
    int column_index = std::clamp(static_cast<int>(pixel_u), 0, static_cast<int>(width) - 1);
    int row_index = std::clamp(static_cast<int>(pixel_v), 0, static_cast<int>(height) - 1);

    float pdf_y = image_sampling.row_probabilities.cdf[row_index + 1]
                  - image_sampling.row_probabilities.cdf[row_index];

    float pdf_x = image_sampling.image_probabilities[row_index].cdf[column_index + 1]
                  - image_sampling.image_probabilities[row_index].cdf[column_index];

    float sin_elevation = std::sin(std::numbers::pi * v);
    float pdf = (pdf_y * pdf_x * width * height)
                / (2.f * std::numbers::pi * std::numbers::pi * sin_elevation);

    return pdf;
  }

  float pdf(const glm::vec3& look_from, const glm::vec3& look_at,
            const glm::vec3& dir) const override {
    return 0.f;
  }

  bool is_emissive() const override { return true; }

  bool is_background() const override { return true; }
};
