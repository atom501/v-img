#include <color_utils.h>

float largest_luminance(const std::vector<glm::vec3>& input_col) {
  float largest_L = 0.0f;

  for (const glm::vec3& col : input_col) {
    float curr_lum = luminance(col);

    if (curr_lum > largest_L) largest_L = curr_lum;
  }

  return largest_L;
}

// changes the color by new luminance
static inline glm::vec3 change_luminance(const glm::vec3& c_in, float l_out) {
  float l_in = luminance(c_in);

  if (l_in > 0.f) {
    return c_in * (l_out / l_in);
  } else {
    return glm::vec3(0.f);
  }
}

void reinhard_lum(std::vector<glm::vec3>& input_col) {
  const float largest_L = largest_luminance(input_col);

  for (glm::vec3& col : input_col) {
    float in_L = luminance(col);
    float numerator = in_L * (1.0f + (in_L / (largest_L * largest_L)));
    float new_L = numerator / (1.0f + in_L);
    col = change_luminance(col, new_L);
  }
}