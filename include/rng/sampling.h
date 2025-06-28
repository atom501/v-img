#pragma once

#include <rng/pcg_rand.h>

#include <algorithm>
#include <cmath>
#include <glm/vec3.hpp>

// give two random numbers [0,1] as input and return a point on sphere.
// where pole of hemisphere is (0,0,1)
inline glm::vec3 sample_sphere(const float& rand1, const float& rand2) {
  float phi = 2 * M_PI * rand1;
  float cos_theta = 2 * rand2 - 1;
  float sin_theta = sqrt(1 - cos_theta * cos_theta);

  float x = cos(phi) * sin_theta;
  float y = sin(phi) * sin_theta;
  float z = cos_theta;

  return glm::vec3(x, y, z);
}

static inline float lerp(float a, float b, float f) { return a + f * (b - a); }

// give two random numbers [0,1] as input and return a point on sphere cap.
// where pole of hemisphere is (0,0,1) and cos_theta_max starts from pole
inline glm::vec3 sample_sphere_cap(const float& rand1, const float& rand2,
                                   const float& cos_theta_max) {
  float phi = 2 * M_PI * rand1;
  float cos_theta = lerp(cos_theta_max, 1.0f, rand2);
  float sin_theta = sqrtf(1 - cos_theta * cos_theta);

  float x = cos(phi) * sin_theta;
  float y = sin(phi) * sin_theta;
  float z = cos_theta;

  return glm::vec3(x, y, z);
}

// give two random numbers [0,1] as input and return a point on hemisphere.
// where pole of hemisphere is (0,0,1)
inline glm::vec3 sample_hemisphere(const float& rand1, const float& rand2) {
  float phi = 2 * M_PI * rand1;
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
  float phi = 2 * M_PI * rand1;
  float cos_theta = std::sqrt(rand2);
  float sin_theta = std::sqrt(1 - cos_theta * cos_theta);

  float x = std::cos(phi) * sin_theta;
  float y = std::sin(phi) * sin_theta;
  float z = cos_theta;

  return glm::vec3(x, y, z);
}

// get a random float of value [0,1)
inline float rand_float(pcg32_random_t& pcg_state) {
  return std::ldexp(static_cast<float>(pcg32_random_r(&pcg_state)), -32);
}

class ArraySampling1D {
public:
  std::vector<float> cdf;
  float funcInt;  // The integral of the absolute value of the function

public:
  // input value given by a function, will be used to create a CDF
  ArraySampling1D(const std::vector<float>& function_values) {
    size_t n = function_values.size();
    ArraySampling1D::cdf = std::vector<float>(n + 1);

    cdf[0] = 0.f;

    // the integral of abs(f) at each point at x
    for (size_t x = 1; x < n + 1; x++) {
      // assumption is that min cdf is 0 and max 1
      cdf[x] = cdf[x - 1] + std::abs(function_values[x - 1]) * 1.f / n;
    }

    // value of the integral
    ArraySampling1D::funcInt = cdf[n];

    // normalize CDF

    // case of uniform probability distribution
    if (ArraySampling1D::funcInt == 0)
      for (size_t i = 1; i < n + 1; ++i) cdf[i] = static_cast<float>(i) / static_cast<float>(n);
    else
      for (size_t i = 1; i < n + 1; ++i) cdf[i] /= ArraySampling1D::funcInt;
  }

  ~ArraySampling1D() = default;

  /*
   * given a random value u [0,1). return the index chosen (index of function_values used to create
   * it) as float
   */
  float sample(float u) {
    // largest index where the CDF was less than or equal to u
    // since will never be 1.f. Maximum index is n - 1 (last value for function_values)
    std::vector<float>::iterator itr = std::upper_bound(cdf.begin(), cdf.end(), u);
    size_t index = itr - cdf.begin() - 1;

    // adding offset for sampling to avoid aliasing
    float du = u - cdf[index];
    if (cdf[index + 1] - cdf[index] > 0) du /= cdf[index + 1] - cdf[index];

    return static_cast<float>(index) + du;
  }
};
