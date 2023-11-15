#pragma once

#include <glm/glm.hpp>
#include <glm/vec3.hpp>
#include <vector>

// Input image given. Image changed by tone mapper
void reinhard_tonemapper(std::vector<glm::vec3>& input_col);

void aces_approx(std::vector<glm::vec3>& input_col);

inline void simple_gamma_correction(std::vector<glm::vec3>& input_col) {
  for (glm::vec3& pixel : input_col) {
    pixel = glm::sqrt(pixel);
  }
}