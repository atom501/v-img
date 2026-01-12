#pragma once

#include <fmt/core.h>
#include <material/disney_helpers/disney_common.h>

#include <algorithm>
#include <numbers>

inline float FD(const glm::vec3& n, const glm::vec3& w, const float FD_90) {
  return 1.f + (FD_90 - 1.f) * std::pow(1.f - std::max(glm::dot(n, w), 0.f), 5.0f);
}

inline glm::vec3 eval_disney_diffuse(const glm::vec3& dir_in, const glm::vec3& dir_out,
                                     const HitInfo& hit, const glm::vec3& base_col,
                                     float subsurface, float roughness, glm::vec3 half_vec,
                                     ONB normal_frame) {
  if (glm::dot(hit.hit_n_g, dir_in) < 0 || glm::dot(hit.hit_n_g, dir_out) < 0) {
    // No light below the surface
    return glm::vec3(0);
  }

  const float cos_theta_out = std::max(glm::dot(normal_frame.w, dir_out), 0.f);
  const float cos_theta_in = std::max(glm::dot(normal_frame.w, dir_in), 0.f);
  const float dot_h_out = std::max(glm::dot(half_vec, dir_out), 0.f);
  const float FD_90 = 0.5 + 2.0 * roughness * dot_h_out * dot_h_out;

  const glm::vec3 base_diffuse = base_col * std::numbers::inv_pi_v<float>
                                 * FD(normal_frame.w, dir_in, FD_90)
                                 * FD(normal_frame.w, dir_out, FD_90) * cos_theta_out;

  const float FSS_90 = roughness * dot_h_out * dot_h_out;

  glm::vec3 ss_diffuse = base_col * 1.25f * std::numbers::inv_pi_v<float>
                         * (FD(normal_frame.w, dir_in, FSS_90) * FD(normal_frame.w, dir_out, FSS_90)
                                * ((1.f / (cos_theta_out + cos_theta_in)) - 0.5f)
                            + 0.5f)
                         * cos_theta_out;

  return (1.f - subsurface) * base_diffuse + subsurface * ss_diffuse;
}

inline float pdf_disney_diffuse(const glm::vec3& dir_in, const glm::vec3& dir_out,
                                const HitInfo& hit, ONB normal_frame) {
  if (glm::dot(hit.hit_n_g, dir_in) < 0 || glm::dot(hit.hit_n_g, dir_out) < 0) {
    // No light below the surface
    return 0;
  }

  return std::max(glm::dot(normal_frame.w, dir_out), 0.f) * std::numbers::inv_pi;
}

inline std::optional<ScatterInfo> sample_disney_diffuse(const glm::vec3& dir_in, const HitInfo& hit,
                                                        ONB normal_frame, pcg32_random_t& pcg_rng) {
  if (glm::dot(hit.hit_n_g, dir_in) < 0) {
    // Incoming direction is below the surface.
    return std::nullopt;
  }

  float rand1 = rand_float(pcg_rng);
  float rand2 = rand_float(pcg_rng);

  glm::vec3 dir_out = xform_with_onb(normal_frame, sample_hemisphere_cosine(rand1, rand2));

  // check if out dir is below the surface
  if (glm::dot(hit.hit_n_g, dir_out) <= 0) {
    return std::nullopt;
  } else {
    return ScatterInfo{dir_out, 0.f, false};
  }
}

inline std::pair<glm::vec3, float> eval_pdf_disney_diffuse(const glm::vec3& dir_in,
                                                           const glm::vec3& dir_out,
                                                           const HitInfo& hit,
                                                           const glm::vec3& base_col,
                                                           float subsurface, float roughness,
                                                           glm::vec3 half_vec, ONB normal_frame) {
  if (glm::dot(hit.hit_n_g, dir_in) < 0 || glm::dot(hit.hit_n_g, dir_out) < 0) {
    // No light below the surface
    return std::make_pair(glm::vec3{0.f, 0.f, 0.f}, 0.f);
  }

  float normal_dirout_dot = glm::dot(normal_frame.w, dir_out);

  const float cos_theta_out = std::max(normal_dirout_dot, 0.f);
  const float cos_theta_in = std::max(glm::dot(normal_frame.w, dir_in), 0.f);
  const float dot_h_out = std::max(glm::dot(half_vec, dir_out), 0.f);
  const float FD_90 = 0.5 + 2.0 * roughness * dot_h_out * dot_h_out;

  const glm::vec3 base_diffuse = base_col * std::numbers::inv_pi_v<float>
                                 * FD(normal_frame.w, dir_in, FD_90)
                                 * FD(normal_frame.w, dir_out, FD_90) * cos_theta_out;

  const float FSS_90 = roughness * dot_h_out * dot_h_out;

  glm::vec3 ss_diffuse = base_col * 1.25f * std::numbers::inv_pi_v<float>
                         * (FD(normal_frame.w, dir_in, FSS_90) * FD(normal_frame.w, dir_out, FSS_90)
                                * ((1.f / (cos_theta_out + cos_theta_in)) - 0.5f)
                            + 0.5f)
                         * cos_theta_out;

  return std::make_pair((1.f - subsurface) * base_diffuse + subsurface * ss_diffuse,
                        std::max(normal_dirout_dot, 0.f) * std::numbers::inv_pi_v<float>);
}
