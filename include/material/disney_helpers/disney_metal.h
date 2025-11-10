#pragma once

#include <color_utils.h>
#include <material/disney_helpers/disney_common.h>

#include <algorithm>
#include <numbers>

inline glm::vec3 eval_disney_metal(const glm::vec3& dir_in, const glm::vec3& dir_out,
                                   const HitInfo& hit, const glm::vec3& base_col, float spec_tint,
                                   float specular, float eta, float metallic, float roughness,
                                   float anisotropic, glm::vec3 half_vec, ONB normal_frame) {
  if (dot(hit.hit_n_g, dir_in) < 0 || dot(hit.hit_n_g, dir_out) < 0) {
    return glm::vec3{0, 0, 0};
  }

  float base_lum = luminance(base_col);
  glm::vec3 C_tint = base_lum > 0 ? base_col / base_lum : glm::vec3(1.f);
  glm::vec3 K_s = (glm::vec3(1.f) - glm::vec3(spec_tint)) + spec_tint * C_tint;
  float R0 = ((eta - 1.f) * (eta - 1.f)) / ((eta + 1.f) * (eta + 1.f));

  glm::vec3 C_0 = (specular * R0 * (1.f - metallic)) * K_s + metallic * base_col;

  glm::vec3 Fresnel
      = C_0 + (glm::vec3(1.f) - C_0) * std::pow((1.f - glm::dot(half_vec, dir_out)), 5.f);

  // get normal distribution or D
  constexpr float alpha_min = 0.0001;
  float aspect = std::sqrt(1.f - 0.9f * anisotropic);
  float roughness_square = roughness * roughness;

  const float alphax = std::max(alpha_min, roughness_square / aspect);
  const float alphay = std::max(alpha_min, roughness_square * aspect);

  const glm::vec3 local_H = project_onto_onb(normal_frame, half_vec);

  float h_alpha_denominator = (local_H.x * local_H.x) / (alphax * alphax)
                              + (local_H.y * local_H.y) / (alphay * alphay)
                              + (local_H.z * local_H.z);

  float D = 1. / (std::numbers::pi * alphax * alphay * (h_alpha_denominator * h_alpha_denominator));

  // get shading, Smith model
  float G = G_w(dir_in, alphax, alphay, normal_frame) * G_w(dir_out, alphax, alphay, normal_frame);

  return (Fresnel * D * G) / (4.f * std::abs(glm::dot(normal_frame.w, dir_in)));
}

inline float pdf_disney_metal(const glm::vec3& dir_in, const glm::vec3& dir_out, const HitInfo& hit,
                              float roughness, float anisotropic, glm::vec3 half_vec,
                              ONB normal_frame) {
  if (dot(hit.hit_n_g, dir_in) < 0 || dot(hit.hit_n_g, dir_out) < 0) {
    return 0.f;
  }

  // get normal distribution or D
  constexpr float alpha_min = 0.0001;
  float aspect = std::sqrt(1.f - 0.9f * anisotropic);
  float roughness_square = roughness * roughness;

  const float alphax = std::max(alpha_min, roughness_square / aspect);
  const float alphay = std::max(alpha_min, roughness_square * aspect);

  const glm::vec3 local_H = project_onto_onb(normal_frame, half_vec);

  float h_alpha_denominator = (local_H.x * local_H.x) / (alphax * alphax)
                              + (local_H.y * local_H.y) / (alphay * alphay)
                              + (local_H.z * local_H.z);

  float D = 1. / (std::numbers::pi * alphax * alphay * (h_alpha_denominator * h_alpha_denominator));

  // get shading, Smith model
  float G_in = G_w(dir_in, alphax, alphay, normal_frame);

  return (D * G_in) / (4.f * std::abs(glm::dot(normal_frame.w, dir_in)));
}

inline std::optional<ScatterInfo> sample_disney_metal(const glm::vec3& dir_in, const HitInfo& hit,
                                                      float roughness, float anisotropic,
                                                      ONB normal_frame, pcg32_random_t& pcg_rng) {
  if (glm::dot(hit.hit_n_g, dir_in) < 0) {
    // No light below the surface
    return std::nullopt;
  }

  // Sample from the specular lobe.

  // Convert the incoming direction to local coordinates
  glm::vec3 local_dir_in = project_onto_onb(normal_frame, dir_in);

  constexpr float alpha_min = 0.0001;
  float aspect = std::sqrt(1.f - 0.9f * anisotropic);
  float roughness_square = roughness * roughness;

  const float alphax = std::max(alpha_min, roughness_square / aspect);
  const float alphay = std::max(alpha_min, roughness_square * aspect);

  glm::vec3 local_micro_normal
      = anisotropic_sample_visible_normals(local_dir_in, alphax, alphay, pcg_rng);

  // Transform the micro normal to world space
  glm::vec3 half_vector = glm::normalize(xform_with_onb(normal_frame, local_micro_normal));
  // Reflect over the world space normal
  glm::vec3 reflected = normalize(-dir_in + 2 * dot(dir_in, half_vector) * half_vector);

  if (glm::dot(reflected, hit.hit_n_g) < 0) {
    return std::nullopt;
  } else {
    return ScatterInfo{reflected, 0.f};
  }
}

inline std::pair<glm::vec3, float> eval_pdf_disney_metal(
    const glm::vec3& dir_in, const glm::vec3& dir_out, const HitInfo& hit,
    const glm::vec3& base_col, float spec_tint, float specular, float eta, float metallic,
    glm::vec3 half_vec, ONB normal_frame, const float G, const float G_in, const float alphax,
    const float alphay) {
  if (dot(hit.hit_n_g, dir_in) < 0 || dot(hit.hit_n_g, dir_out) < 0) {
    return std::make_pair(glm::vec3{0.f, 0.f, 0.f}, 0.f);
  }

  glm::vec3 eval = glm::vec3(0.f);
  float pdf = 0.f;

  float base_lum = luminance(base_col);
  glm::vec3 C_tint = base_lum > 0 ? base_col / base_lum : glm::vec3(1.f);
  glm::vec3 K_s = (glm::vec3(1.f) - glm::vec3(spec_tint)) + spec_tint * C_tint;
  float R0 = ((eta - 1.f) * (eta - 1.f)) / ((eta + 1.f) * (eta + 1.f));

  glm::vec3 C_0 = (specular * R0 * (1.f - metallic)) * K_s + metallic * base_col;

  glm::vec3 Fresnel
      = C_0 + (glm::vec3(1.f) - C_0) * std::pow((1.f - glm::dot(half_vec, dir_out)), 5.f);

  const glm::vec3 local_H = project_onto_onb(normal_frame, half_vec);

  float h_alpha_denominator = (local_H.x * local_H.x) / (alphax * alphax)
                              + (local_H.y * local_H.y) / (alphay * alphay)
                              + (local_H.z * local_H.z);

  float D = 1. / (std::numbers::pi * alphax * alphay * (h_alpha_denominator * h_alpha_denominator));

  float D_mul_denominator = D / (4.f * std::abs(glm::dot(normal_frame.w, dir_in)));

  return std::make_pair(Fresnel * G * D_mul_denominator, G_in * D_mul_denominator);
}