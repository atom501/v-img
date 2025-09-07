#pragma once

#include <glm/glm.hpp>
#include <glm/vec3.hpp>
#include <vector>

enum class tonemapper { clamp, agx, COUNT };

static inline float luminance(const glm::vec3& v) {
  return glm::dot(v, glm::vec3(0.212671f, 0.715160f, 0.072169f));
}

// Input image given. Image changed by tone mapper
void reinhard_tonemapper(std::vector<glm::vec3>& input_col);

void aces_approx(std::vector<glm::vec3>& input_col);

void agx(std::vector<glm::vec3>& input_col);

// clamping colors to [0,1]
inline void simple_clamp(std::vector<glm::vec3>& input_col) {
  for (glm::vec3& col : input_col) {
    col = glm::clamp(col, 0.f, 1.f);
  }
}

inline void sRGB_gamma_correction(std::vector<glm::vec3>& input_col) {
  for (glm::vec3& pixel : input_col) {
    pixel = glm::clamp(pixel, 0.0f, 1.0f);

    if (pixel.x < 0.0031308f)
      pixel.x *= 12.92f;
    else
      pixel.x = 1.055f * std::pow(pixel.x, 1.0f / 2.4f) - 0.055f;

    if (pixel.y < 0.0031308f)
      pixel.y *= 12.92f;
    else
      pixel.y = 1.055f * std::pow(pixel.y, 1.0f / 2.4f) - 0.055f;

    if (pixel.z < 0.0031308f)
      pixel.z *= 12.92f;
    else
      pixel.z = 1.055f * std::pow(pixel.z, 1.0f / 2.4f) - 0.055f;
  }
}