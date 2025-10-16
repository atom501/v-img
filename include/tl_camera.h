#pragma once

#include <ray.h>

#include <cstdint>

#include "glm/mat4x4.hpp"

// Thin lens camera
class TLCam {
private:
  float vfov;                                    // Vertical FOV. In degrees
  glm::mat4x4 camToWorld_xform;                  // Camera to world transform
  glm::vec4 p_size = glm::vec4(1.0f);            // Physical size of the image plane
  glm::ivec2 resolution = glm::ivec2(512, 512);  // Resolution

  // TODO depth of field
  // float aperture_radius = 0.f;                   // The size of the aperture for depth of field
  // float focal_dist = 1.f;                        // Distance to image plane along cam z axis

public:
  TLCam(const glm::mat4& xform, const glm::ivec2& res, const float ver_fov);
  TLCam() = default;
  ~TLCam() = default;

  // generate ray with direction normalized
  Ray generate_ray(const float& x, const float& y) const;
};
/*
  Gives camera to world transform
  lookFrom: position of center of camera
  lookAt: point camera is looking at
  up: up direction for camera
*/
glm::mat4 camToWorld(const glm::vec3& lookFrom, const glm::vec3& lookAT, const glm::vec3& up);