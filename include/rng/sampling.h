#pragma once

#include <glm/vec3.hpp>
#include <numbers>

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
