#pragma once

#include <material/disney_helpers/disney_common.h>
#include <material/disney_helpers/disney_diffuse.h>

#include <algorithm>
#include <numbers>

inline glm::vec3 eval_disney_clearcoat(const glm::vec3& dir_in, const glm::vec3& dir_out,
                                       const HitInfo& hit, const glm::vec3& base_col,
                                       float clearcoat_gloss, const glm::vec3& half_vec,
                                       ONB normal_frame) {
  if (glm::dot(hit.hit_n_g, dir_in) < 0 || glm::dot(hit.hit_n_g, dir_out) < 0) {
    // No light below the surface
    return glm::vec3(0);
  }

  // index refraction with ior 1.5 fixed
  constexpr float R0 = ((1.5f - 1.f) * (1.5f - 1.f)) / ((1.5f + 1.f) * (1.5f + 1.f));

  float Fresenl = R0 + (1. - R0) * std::pow(1.f - std::abs(glm::dot(half_vec, dir_out)), 5.f);

  float G = G_w(dir_in, 0.25, 0.25, normal_frame) * G_w(dir_out, 0.25, 0.25, normal_frame);

  const float alpha_g = (1.f - clearcoat_gloss) * 0.1f + clearcoat_gloss * 0.001f;
  const float alpha_g_square = alpha_g * alpha_g;

  const glm::vec3 local_H = project_onto_onb(normal_frame, half_vec);

  float D = (alpha_g_square - 1.f)
            / (std::numbers::pi * std::log2(alpha_g_square)
               * (1. + (alpha_g_square - 1.) * local_H.z * local_H.z));

  float clearcoat_eval = (Fresenl * D * G) / (4.f * std::abs(glm::dot(normal_frame.w, dir_in)));

  return glm::vec3(clearcoat_eval);
}

inline float pdf_disney_clearcoat(const glm::vec3& dir_in, const glm::vec3& dir_out,
                                  const HitInfo& hit, const glm::vec3& base_col,
                                  float clearcoat_gloss, const glm::vec3& half_vec,
                                  ONB normal_frame) {
  if (glm::dot(hit.hit_n_g, dir_in) < 0 || glm::dot(hit.hit_n_g, dir_out) < 0) {
    // No light below the surface
    return 0.f;
  }

  const float alpha_g = (1.f - clearcoat_gloss) * 0.1f + clearcoat_gloss * 0.001f;
  const float alpha_g_square = alpha_g * alpha_g;

  const glm::vec3 local_H = project_onto_onb(normal_frame, half_vec);

  float D = (alpha_g_square - 1.f)
            / (std::numbers::pi * std::log2(alpha_g_square)
               * (1.f + (alpha_g_square - 1.f) * local_H.z * local_H.z));

  return (D * std::abs(glm::dot(normal_frame.w, half_vec)))
         / (4.f * std::abs(glm::dot(half_vec, dir_out)));
}

inline glm::vec3 sample_local_h_clearcoat(pcg32_random_t& pcg_rng, const float alpha) {
  float rand1 = rand_float(pcg_rng);
  float rand2 = rand_float(pcg_rng);

  float cos_square_elevation
      = (1.f - std::pow((alpha * alpha), 1. - rand1)) / (1.f - (alpha * alpha));
  float cos_elevation = std::sqrt(cos_square_elevation);
  float sin_elevation = std::sqrt(1 - cos_square_elevation);
  float h_azimuth = 2.f * std::numbers::pi * rand2;
  float x = sin_elevation * std::cos(h_azimuth);
  float y = sin_elevation * std::sin(h_azimuth);
  float z = cos_elevation;

  return glm::vec3(x, y, z);
}

inline std::optional<ScatterInfo> sample_disney_clearcoat(const glm::vec3& dir_in,
                                                          const HitInfo& hit, ONB normal_frame,
                                                          float clearcoat_gloss,
                                                          pcg32_random_t& pcg_rng) {
  if (glm::dot(hit.hit_n_g, dir_in) < 0) {
    // No light below the surface
    return std::nullopt;
  }

  const float alpha_g = (1.f - clearcoat_gloss) * 0.1f + clearcoat_gloss * 0.001f;
  glm::vec3 local_h = sample_local_h_clearcoat(pcg_rng, alpha_g);

  if (glm::dot(normal_frame.w, dir_in) < 0) {
    normal_frame.u = -normal_frame.u;
    normal_frame.v = -normal_frame.v;
    normal_frame.w = -normal_frame.w;
  }

  const glm::vec3 H = glm::normalize(xform_with_onb(normal_frame, local_h));
  glm::vec3 reflected = glm::normalize(-dir_in + 2 * glm::dot(dir_in, H) * H);

  if (glm::dot(hit.hit_n_g, reflected) < 0) {
    return std::nullopt;
  } else {
    return ScatterInfo{reflected, 0};
  }
}