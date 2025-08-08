#pragma once

#include <material/material.h>
#include <rng/sampling.h>

inline float G_w(const glm::vec3& w, float alphax, float alphay, const ONB& frame) {
  const glm::vec3 w_local = project_onto_onb(frame, w);
  float vec_alpha
      = ((w_local.x * alphax) * (w_local.x * alphax) + (w_local.y * alphay) * (w_local.y * alphay))
        / (w_local.z * w_local.z);
  float caret = (std::sqrt(1. + (vec_alpha)) - 1.) / 2.;

  return 1. / (1. + caret);
}