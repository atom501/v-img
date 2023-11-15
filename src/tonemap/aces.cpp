#include <tonemapper.h>

#include <algorithm>

void aces_approx(std::vector<glm::vec3>& input_col) {
  for (glm::vec3& pixel : input_col) {
    pixel *= 0.6f;
    float a = 2.51f;
    float b = 0.03f;
    float c = 2.43f;
    float d = 0.59f;
    float e = 0.14f;
    pixel = glm::clamp((pixel * (a * pixel + b)) / (pixel * (c * pixel + d) + e), 0.0f, 1.0f);
  }
}