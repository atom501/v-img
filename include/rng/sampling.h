#pragma once

#include <rng/pcg_rand.h>

#include <algorithm>
#include <bit>
#include <cmath>
#include <glm/vec3.hpp>
#include <span>
#include <vector>
#include <numbers>

// give two random numbers [0,1] as input and return a point on sphere.
// where pole of hemisphere is (0,0,1)
inline glm::vec3 sample_sphere(const float& rand1, const float& rand2) {
  float phi = 2 * std::numbers::pi * rand1;
  float cos_theta = 2 * rand2 - 1;
  float sin_theta = sqrt(1 - cos_theta * cos_theta);

  float x = cos(phi) * sin_theta;
  float y = sin(phi) * sin_theta;
  float z = cos_theta;

  return glm::vec3(x, y, z);
}

// give two random numbers [0,1] as input and return a point on sphere cap.
// where pole of hemisphere is (0,0,1) and cos_theta_max starts from pole
inline glm::vec3 sample_sphere_cap(const float& rand1, const float& rand2,
                                   const float& cos_theta_max) {
  float phi = 2 * std::numbers::pi * rand1;
  float cos_theta = std::lerp(cos_theta_max, 1.0f, rand2);
  float sin_theta = sqrtf(1 - cos_theta * cos_theta);

  float x = cos(phi) * sin_theta;
  float y = sin(phi) * sin_theta;
  float z = cos_theta;

  return glm::vec3(x, y, z);
}

// give two random numbers [0,1] as input and return a point on hemisphere.
// where pole of hemisphere is (0,0,1)
inline glm::vec3 sample_hemisphere(const float& rand1, const float& rand2) {
  float phi = 2 * std::numbers::pi * rand1;
  float cos_theta = rand2;
  float sin_theta = std::sqrt(1 - cos_theta * cos_theta);

  float x = std::cos(phi) * sin_theta;
  float y = std::sin(phi) * sin_theta;
  float z = cos_theta;

  return glm::vec3(x, y, z);
}

/// input two random numbers and return Cosine-weighted point on the hemisphere
/// where pole of hemisphere is (0,0,1)
inline glm::vec3 sample_hemisphere_cosine(const float& rand1, const float& rand2) {
  float phi = 2 * std::numbers::pi * rand1;
  float cos_theta = std::sqrt(rand2);
  float sin_theta = std::sqrt(1 - cos_theta * cos_theta);

  float x = std::cos(phi) * sin_theta;
  float y = std::sin(phi) * sin_theta;
  float z = cos_theta;

  return glm::vec3(x, y, z);
}

/*
 * get a random float of value [0,1). source:
 * https://marc-b-reynolds.github.io/distribution/2017/01/17/DenseFloat.html
 */
inline float rand_float(pcg32_random_t& pcg_state) {
  uint64_t r1 = pcg32_random_r(&pcg_state);
  uint64_t r2 = pcg32_random_r(&pcg_state);

  uint64_t u = (r1 << 32ull) | r2;

  uint32_t z = std::countl_zero(u);

  if (z <= 40) {
    uint32_t e = 126 - z;                   // compute the biased exponent
    uint32_t m = ((uint32_t)u) & 0x7fffff;  // explict significand bits
    uint32_t float_bits = e << 23 | m;
    return reinterpret_cast<float&>(float_bits);  // construct the binary32
  }

  // The probabilty of reaching here is 2^-40. There are as many points
  // on this subinterval as the standard equidistance method produces
  // across the entire output range.

  return 0x1.0p-64f * (float)((uint32_t)u);
}

class ArraySampling1D {
public:
  std::vector<float> cdf;
  float func_int;  // The integral of the absolute value of the function

public:
  // input value given by a function, will be used to create a CDF
  ArraySampling1D(std::span<const float> function_values) {
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

  ArraySampling1D() = default;
  ~ArraySampling1D() = default;

  /*
   * given a random value u [0,1). return the index chosen (index of function_values used to create
   * it) and offset as (index, offset)
   */
  std::pair<size_t, float> sample(float u) const {
    // largest index where the CDF was less than or equal to u
    // since will never be 1.f. Maximum index is n - 1 (last value for function_values)
    std::vector<float>::const_iterator itr = std::upper_bound(cdf.begin(), cdf.end(), u);
    size_t index = itr - cdf.begin() - 1;

    // adding offset for sampling to avoid aliasing
    float du = u - cdf[index];
    if (cdf[index + 1] - cdf[index] > 0) du /= cdf[index + 1] - cdf[index];

    return std::make_pair(index, du);
  }
};

class ArraySampling2D {
public:
  ArraySampling1D row_probabilities;
  std::vector<ArraySampling1D> image_probabilities;
  uint32_t width;
  uint32_t height;

  ArraySampling2D() = default;

  ArraySampling2D(const std::vector<glm::vec3>& image, uint32_t width, uint32_t height) {
    ArraySampling2D::width = width;
    ArraySampling2D::height = height;

    // make a copy of image with luminance values
    std::vector<float> image_luminance(image.size());

    constexpr glm::vec3 lum_const = glm::vec3(0.2126f, 0.7152f, 0.0722f);
    for (size_t y = 0; y < height; y++) {
      float v = (static_cast<float>(y) + 0.5f) / static_cast<float>(height);
      float sin_elevation = std::sin(std::numbers::pi * v);

      for (size_t x = 0; x < width; x++) {
        image_luminance[(y * width) + x]
            = glm::dot(image[(y * width) + x], lum_const) * sin_elevation;
      }
    }

    std::vector<float> row_integral_vals;

    for (size_t h = 0; h < height; h++) {
      std::span<const float> img_span(image_luminance.data() + (h * width), width);

      const ArraySampling1D prob_sampling_of_row = ArraySampling1D(img_span);
      image_probabilities.push_back(prob_sampling_of_row);
      row_integral_vals.push_back(prob_sampling_of_row.func_int);
    }

    // using integral values of each row as weights
    row_probabilities = ArraySampling1D(row_integral_vals);
  }

  ~ArraySampling2D() = default;

  /*
   * return the (u, v) coords of the env map. where (0, 0) is the top left corner. Third item is the
   * pdf of choosing the sample
   */
  std::tuple<float, float, float> sample(float r1, float r2) const {
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
};
