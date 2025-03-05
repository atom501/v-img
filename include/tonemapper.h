#pragma once

#include <glm/glm.hpp>
#include <glm/vec3.hpp>
#include <vector>

// Input image given. Image changed by tone mapper
void reinhard_tonemapper(std::vector<glm::vec3>& input_col);

void aces_approx(std::vector<glm::vec3>& input_col);

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