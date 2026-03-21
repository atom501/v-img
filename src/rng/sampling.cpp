#include <color_utils.h>
#include <rng/sampling.h>

#include <algorithm>

ArraySampling1D::ArraySampling1D(std::span<const float> function_values) {
  size_t n = function_values.size();
  ArraySampling1D::cdf = std::vector<float>(n + 1);

  cdf[0] = 0.f;

  // the integral of abs(f) at each point at x
  for (size_t x = 1; x < n + 1; x++) {
    // assumption is that min cdf is 0 and max 1
    cdf[x] = cdf[x - 1] + std::abs(function_values[x - 1]);
  }

  // value of the integral
  ArraySampling1D::func_int = cdf[n];

  // normalize CDF
  // case of uniform probability distribution
  if (ArraySampling1D::func_int == 0)
    for (size_t i = 0; i < n + 1; ++i) cdf[i] = static_cast<float>(i) / static_cast<float>(n);
  else
    for (size_t i = 0; i < n + 1; ++i) cdf[i] /= ArraySampling1D::func_int;
}

std::pair<size_t, float> ArraySampling1D::sample(float u) const {
  // largest index where the CDF was less than or equal to u
  // since will never be 1.f. Maximum index is n - 1 (last value for function_values)
  std::vector<float>::const_iterator itr = std::upper_bound(cdf.begin(), cdf.end(), u);
  size_t index = itr - cdf.begin() - 1;

  // adding offset for sampling to avoid aliasing
  float du = u - cdf[index];
  if (cdf[index + 1] - cdf[index] > 0) du /= cdf[index + 1] - cdf[index];

  return std::make_pair(index, du);
}

ArraySampling2D::ArraySampling2D(const std::vector<glm::vec3>& image, uint32_t width,
                                 uint32_t height) {
  ArraySampling2D::width = width;
  ArraySampling2D::height = height;

  // make a copy of image with luminance values
  std::vector<float> image_luminance(image.size());

  for (size_t y = 0; y < height; y++) {
    float v = (static_cast<float>(y) + 0.5f) / static_cast<float>(height);
    float sin_elevation = std::sin(std::numbers::pi * v);

    for (size_t x = 0; x < width; x++) {
      image_luminance[(y * width) + x] = luminance(image[(y * width) + x]) * sin_elevation;
    }
  }

  std::vector<float> row_integral_vals(height);
  image_probabilities = std::vector<ArraySampling1D>(height);

  for (size_t h = 0; h < height; h++) {
    std::span<const float> img_span(image_luminance.data() + (h * width), width);

    const ArraySampling1D prob_sampling_of_row = ArraySampling1D(img_span);

    image_probabilities[h] = prob_sampling_of_row;
    row_integral_vals[h] = prob_sampling_of_row.func_int;
  }

  // using integral values of each row as weights
  row_probabilities = ArraySampling1D(row_integral_vals);
}

std::tuple<float, float, float> ArraySampling2D::sample(float r1, float r2) const {
  // pick a row
  auto [row_index, dv] = row_probabilities.sample(r1);

  // pick column
  auto [column_index, du] = image_probabilities[row_index].sample(r2);

  const float u = (static_cast<float>(column_index) + du) / ArraySampling2D::width;
  const float v = (static_cast<float>(row_index) + dv) / ArraySampling2D::height;

  float pdf_y = row_probabilities.cdf[row_index + 1] - row_probabilities.cdf[row_index];

  float pdf_x = image_probabilities[row_index].cdf[column_index + 1]
                - image_probabilities[row_index].cdf[column_index];
  float pdf = pdf_y * pdf_x;

  return std::make_tuple(u, v, pdf);
}