#pragma once

#include <rng/pcg_rand.h>

#include <bit>
#include <cmath>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <numbers>
#include <span>
#include <vector>

// give two random numbers [0,1] as input and return a point on disk on with radius 1.
inline glm::vec2 sample_disk(const float& rand1, const float& rand2) {
  float r = std::sqrt(rand1);
  float phi = 2.f * std::numbers::pi * rand2;
  float x = r * std::cos(phi);
  float y = r * std::sin(phi);

  return glm::vec2(x, y);
}

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
    return std::bit_cast<float>(float_bits);  // construct the binary32
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
  ArraySampling1D(std::span<const float> function_values);

  ArraySampling1D() = default;
  ~ArraySampling1D() = default;

  /*
   * given a random value u [0,1). return the index chosen (index of function_values used to create
   * it) and offset as (index, offset)
   */
  std::pair<size_t, float> sample(float u) const;
};

class ArraySampling2D {
public:
  ArraySampling1D row_probabilities;
  std::vector<ArraySampling1D> image_probabilities;
  uint32_t width;
  uint32_t height;

  ArraySampling2D() = default;

  ArraySampling2D(const std::vector<glm::vec3>& image, uint32_t width, uint32_t height);

  ~ArraySampling2D() = default;

  /*
   * return the (u, v) coords of the env map. where (0, 0) is the top left corner. Third item is the
   * pdf of choosing the sample
   */
  std::tuple<float, float, float> sample(float r1, float r2) const;
};

// get number from r2 sequence. returns (x, y) where both are [0, 1)
// source: https://www.martysmods.com/a-better-r2-sequence/

inline glm::vec2 random_x_y_r2(uint32_t n) {
  constexpr float g = 1.32471795724474602596;
  constexpr float a1 = 1.0 - (1.0 / g);
  constexpr float a2 = 1.0 - (1.0 / (g * g));

  float x = a1 * n;
  float y = a2 * n;

  // return fractional part

  return glm::vec2(x - std::floor(x), y - std::floor(y));
}
