#pragma once

#include <comptime_settings.h>
#include <geometry/emitters.h>
#include <ray.h>
#include <texture/texture_RGB.h>

#include <algorithm>
#include <numbers>
#include <utility>
#include <vector>

#include "glm/vec3.hpp"

class Background : public Emitter {
public:
  Background() = default;
  virtual ~Background() = default;

  virtual glm::vec3 background_emit(const glm::vec3& dir, const RayCone& cone) const = 0;
  virtual bool is_emissive() const = 0;
  virtual float background_pdf(const glm::vec3& dir) const = 0;
};

class ConstBackground : public Background {
private:
  glm::vec3 col;

public:
  ConstBackground(const glm::vec3& const_col) : col(const_col) {}
  ~ConstBackground() = default;

  glm::vec3 background_emit(const glm::vec3& dir, const RayCone& cone) const override {
    return col;
  }

  std::pair<glm::vec3, EmitterInfo> sample(const glm::vec3& look_from,
                                           pcg32_random_t& pcg_rng) const override {
    float r1 = rand_float(pcg_rng);
    float r2 = rand_float(pcg_rng);
    // sample uniform sphere
    glm::vec3 wi = sample_sphere(r1, r2);

    constexpr float pdf = 1.f / (4 * std::numbers::pi);

    return std::make_pair(ConstBackground::col,
                          EmitterInfo{wi, pdf, std::numeric_limits<float>::infinity(), 1.f});
  }

  float background_pdf(const glm::vec3& dir) const override { return 1.f / (4 * std::numbers::pi); }

  bool is_emissive() const override {
    if (col == glm::vec3(0.f))
      return false;
    else
      return true;
  }

  bool is_background() const override { return true; }

  float surf_pdf(const glm::vec3& look_from, const glm::vec3& look_at,
                 const glm::vec3& dir) const override {
    return 0.f;
  }
};

// image loaded in from a file
class EnvMap : public Background {
private:
  ImageTexture image;
  glm::mat4 world_to_env;
  glm::mat4 env_to_world;
  ArraySampling2D image_sampling;
  float radiance_scale;

public:
  EnvMap(ImageTexture& image, const glm::mat4& world_to_env, const glm::mat4& env_to_world,
         float radiance_scale)
      : image(image),
        world_to_env(world_to_env),
        env_to_world(env_to_world),
        radiance_scale(radiance_scale) {
    image_sampling = ArraySampling2D(image.mipmap[0], image.width, image.height);
  }
  ~EnvMap() = default;

  // transform ray from world to image space. Then get emitted color
  glm::vec3 background_emit(const glm::vec3& in_dir, const RayCone& cone) const override {
    // transform from ray dir world space to obj/env space
    glm::vec3 dir = glm::vec3(world_to_env * glm::vec4(in_dir, 0.0f));

    // get uv coordinates. Envmap assumes latitude-longitude format. same as in mitsuba
    dir = glm::normalize(dir);
    float u = (1.f + std::atan2(-dir.x, dir.z) * std::numbers::inv_pi) * 0.5f;
    float v = std::acos(dir.y) * std::numbers::inv_pi;

    // calculate mipmap level
    float lambda;
    if constexpr (CompileConsts::mipmap0) {
      lambda = 0.f;
    } else {
      lambda = std::log2(std::abs(cone.spread_angle) * (image.height / std::numbers::pi));
    }

    lambda = std::isnan(lambda) ? 0.f : lambda;

    return image.col_mipmap_interpolate(lambda - 2.f, glm::vec2(u, v)) * radiance_scale;
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
    float pdf = (choose_sample_pdf * image.width * image.height)
                / (2.f * std::numbers::pi * std::numbers::pi * sin_elevation);

    return std::make_pair(image.col_at_uv_mipmap(0, glm::vec2(u_env, v_env)) * radiance_scale,
                          EmitterInfo{wi, pdf, std::numeric_limits<float>::infinity(), 1.f});
  }

  float background_pdf(const glm::vec3& in_dir) const override {
    // transform from ray dir world space to obj/env space
    glm::vec3 dir = glm::vec3(world_to_env * glm::vec4(in_dir, 0.0f));

    // get uv coordinates. Envmap assumes latitude-longitude format. same as in mitsuba
    dir = glm::normalize(dir);
    float u = (1.f + std::atan2(-dir.x, dir.z) * std::numbers::inv_pi) * 0.5f;
    float v = std::acos(dir.y) * std::numbers::inv_pi;

    // use u, v to get pdf
    int pixel_u = u * image.width;
    int pixel_v = v * image.height;

    // get pixel value using sampling
    int column_index = std::clamp(static_cast<int>(pixel_u), 0, static_cast<int>(image.width) - 1);
    int row_index = std::clamp(static_cast<int>(pixel_v), 0, static_cast<int>(image.height) - 1);

    float pdf_y = image_sampling.row_probabilities.cdf[row_index + 1]
                  - image_sampling.row_probabilities.cdf[row_index];

    float pdf_x = image_sampling.image_probabilities[row_index].cdf[column_index + 1]
                  - image_sampling.image_probabilities[row_index].cdf[column_index];

    float sin_elevation = std::sin(std::numbers::pi * v);
    float pdf = (pdf_y * pdf_x * image.width * image.height)
                / (2.f * std::numbers::pi * std::numbers::pi * sin_elevation);

    return pdf;
  }

  float surf_pdf(const glm::vec3& look_from, const glm::vec3& look_at,
                 const glm::vec3& dir) const override {
    return 0.f;
  }

  bool is_emissive() const override { return true; }

  bool is_background() const override { return true; }
};
