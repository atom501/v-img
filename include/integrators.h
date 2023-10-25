#pragma once

#include <geometry/sphere.h>
#include <tl_camera.h>

#include <cstdint>
#include <glm/glm.hpp>
#include <vector>

struct integrator_data {
  glm::ivec2 resolution;
  uint32_t samples;
};

std::vector<glm::vec3> normal_integrator(const integrator_data& render_data, const TLCam& camera,
                                         const Sphere& s);