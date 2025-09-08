#include <color_utils.h>

#include <algorithm>

static inline glm::vec3 rtt_and_odt_fit(glm::vec3 v) {
  glm::vec3 a = v * (v + 0.0245786f) - 0.000090537f;
  glm::vec3 b = v * (0.983729f * v + 0.4329510f) + 0.238081f;
  return a / b;
}

static inline glm::vec3 aces_fitted(glm::vec3 v) {
  constexpr static glm::mat3 aces_input_matrix
      = glm::mat3{glm::vec3(0.59719f, 0.07600f, 0.02840f), glm::vec3(0.35458f, 0.90834f, 0.13383f),
                  glm::vec3(0.04823f, 0.01566f, 0.83777f)};

  constexpr static glm::mat3 aces_output_matrix = glm::mat3{
      glm::vec3(1.60475f, -0.10208f, -0.00327f), glm::vec3(-0.53108f, 1.10813f, -0.07276f),
      glm::vec3(-0.07367f, -0.00605f, 1.07602f)};

  v = aces_input_matrix * v;
  v = rtt_and_odt_fit(v);
  return aces_output_matrix * v;
}

void aces(std::vector<glm::vec3>& input_col) {
  for (glm::vec3& pixel : input_col) {
    pixel = aces_fitted(pixel);
  }
}