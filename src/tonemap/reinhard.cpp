#include <color_utils.h>

float largest_luminance(const std::vector<glm::vec3>& input_col) {
  float largest_L = 0.0f;
  float temp_L;

  for (size_t i = 0; i < input_col.size(); i++) {
    temp_L = luminance(input_col[i]);

    if (temp_L > largest_L) largest_L = temp_L;
  }

  return largest_L;
}

// changes the color by new luminance
static inline void change_luminance(glm::vec3& c_in, float l_out) {
  float l_in = luminance(c_in);
  c_in = c_in * (l_out / l_in);
}

void reinhard_tonemapper(std::vector<glm::vec3>& input_col) {
  const float largest_L = largest_luminance(input_col);

  for (size_t i = 0; i < input_col.size(); i++) {
    float in_L = luminance(input_col[i]);
    float numerator = in_L * (1.0f + (in_L / (largest_L * largest_L)));
    float new_L = numerator / (1.0f + in_L);
    change_luminance(input_col[i], new_L);
  }
}