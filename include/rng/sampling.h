#pragma once

#include <rng/pcg_rand.h>

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
  return ldexp(static_cast<float>(pcg32_random_r(&pcg_state)), -32);
}
