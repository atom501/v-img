#pragma once

#include <background.h>
#include <tl_camera.h>

#include <cstdint>
#include <glm/vec2.hpp>

enum class integrator_func { s_normal, g_normal, material, mis, COUNT };

struct integrator_data {
  integrator_func func;
  glm::ivec2 resolution;
  uint32_t samples;
  uint32_t depth;
  std::unique_ptr<Background> background;
  TLCam camera;
  int16_t num_threads;
};