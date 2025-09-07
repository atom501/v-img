#pragma once

#include <color_utils.h>
#include <material/disney_helpers/disney_common.h>
#include <material/disney_helpers/disney_diffuse.h>

#include <algorithm>
#include <numbers>

inline glm::vec3 eval_disney_sheen(const glm::vec3& dir_in, const glm::vec3& dir_out,
                                   const HitInfo& hit, const glm::vec3& base_col, float sheen_tint,
                                   glm::vec3 half_vec, ONB normal_frame) {
  if (glm::dot(hit.hit_n_g, dir_in) < 0 || glm::dot(hit.hit_n_g, dir_out) < 0) {
    // No light below the surface
    return glm::vec3(0);
  }

  const float base_lum = luminance(base_col);

  glm::vec3 C_tint = base_lum > 0 ? base_col / base_lum : glm::vec3(1.f);

  glm::vec3 C_sheen
      = (glm::vec3(1.f) - glm::vec3(sheen_tint, sheen_tint, sheen_tint)) + sheen_tint * C_tint;

  return C_sheen * std::pow((1.f - std::max(glm::dot(half_vec, dir_out), 0.f)), 5.f)
         * std::max(glm::dot(normal_frame.w, dir_out), 0.f);
}

inline float pdf_disney_sheen(const glm::vec3& dir_in, const glm::vec3& dir_out, const HitInfo& hit,
                              ONB normal_frame) {
  return pdf_disney_diffuse(dir_in, dir_out, hit, normal_frame);
}

inline std::optional<ScatterInfo> sample_disney_sheen(const glm::vec3& dir_in, const HitInfo& hit,
                                                      ONB normal_frame, pcg32_random_t& pcg_rng) {
  return sample_disney_diffuse(dir_in, hit, normal_frame, pcg_rng);
}