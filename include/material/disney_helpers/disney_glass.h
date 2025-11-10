#pragma once

#include <material/disney_helpers/disney_common.h>

#include <algorithm>
#include <numbers>

inline glm::vec3 eval_disney_rough_glass(const glm::vec3& dir_in, const glm::vec3& dir_out,
                                         const HitInfo& hit, const glm::vec3& base_col,
                                         float mat_eta, float anisotropic, float roughness,
                                         glm::vec3 half_vec, ONB normal_frame) {
  float in_geo_dot = glm::dot(dir_in, hit.hit_n_g);

  bool reflect = (in_geo_dot * glm::dot(hit.hit_n_g, dir_out)) >= 0;

  // flip eta when going out of the surface
  float eta = in_geo_dot >= 0 ? mat_eta : 1.f / mat_eta;

  constexpr float alpha_min = 0.0001;
  float aspect = std::sqrt(1.f - 0.9f * anisotropic);
  // Clamp roughness to avoid numerical issues.
  roughness = std::clamp(roughness, 0.01f, 1.f);

  float roughness_square = roughness * roughness;

  const float alphax = std::max(alpha_min, roughness_square / aspect);
  const float alphay = std::max(alpha_min, roughness_square * aspect);

  if (!reflect) {
    // "Generalized half-vector" from Walter et al.
    half_vec = normalize(dir_in + dir_out * eta);
  }

  float h_dot_in = glm::dot(half_vec, dir_in);
  float F = fresnel_dielectric(h_dot_in, eta);

  const glm::vec3 local_H = project_onto_onb(normal_frame, half_vec);
  float h_alpha_denominator = (local_H.x * local_H.x) / (alphax * alphax)
                              + (local_H.y * local_H.y) / (alphay * alphay)
                              + (local_H.z * local_H.z);
  float D = 1. / (std::numbers::pi * alphax * alphay * (h_alpha_denominator * h_alpha_denominator));
  // Smith shadowing model
  float G = G_w(dir_in, alphax, alphay, normal_frame) * G_w(dir_out, alphax, alphay, normal_frame);

  if (reflect) {
    return base_col * (F * D * G) / (4.f * std::abs(glm::dot(normal_frame.w, dir_in)));
  } else {
    float eta_factor = 1.f / (eta * eta);
    float h_dot_out = glm::dot(half_vec, dir_out);
    float sqrt_denom = h_dot_in + eta * h_dot_out;

    return glm::vec3(std::sqrt(base_col.x), std::sqrt(base_col.y), std::sqrt(base_col.z))
           * (eta_factor * (1 - F) * D * G * eta * eta * std::abs(h_dot_out * h_dot_in))
           / (std::abs(glm::dot(normal_frame.w, dir_in)) * sqrt_denom * sqrt_denom);
  }
}

inline float pdf_disney_rough_glass(const glm::vec3& dir_in, const glm::vec3& dir_out,
                                    const HitInfo& hit, float mat_eta, float anisotropic,
                                    float roughness, glm::vec3 half_vec, ONB normal_frame) {
  float in_geo_dot = glm::dot(dir_in, hit.hit_n_g);

  bool reflect = (in_geo_dot * glm::dot(hit.hit_n_g, dir_out)) >= 0;

  // flip eta when going out of the surface
  float eta = in_geo_dot >= 0 ? mat_eta : 1.f / mat_eta;

  if (!reflect) {
    // "Generalized half-vector" from Walter et al.
    half_vec = normalize(dir_in + dir_out * eta);
  }

  constexpr float alpha_min = 0.0001;
  float aspect = std::sqrt(1.f - 0.9f * anisotropic);
  // Clamp roughness to avoid numerical issues.
  roughness = std::clamp(roughness, 0.01f, 1.f);

  float roughness_square = roughness * roughness;

  const float alphax = std::max(alpha_min, roughness_square / aspect);
  const float alphay = std::max(alpha_min, roughness_square * aspect);

  // We sample the visible normals, also we use F to determine
  // whether to sample reflection or refraction
  // so PDF ~ F * D * G_in for reflection, PDF ~ (1 - F) * D * G_in for refraction.
  float h_dot_in = glm::dot(half_vec, dir_in);
  float F = fresnel_dielectric(h_dot_in, eta);

  const glm::vec3 local_H = project_onto_onb(normal_frame, half_vec);
  float h_alpha_denominator = (local_H.x * local_H.x) / (alphax * alphax)
                              + (local_H.y * local_H.y) / (alphay * alphay)
                              + (local_H.z * local_H.z);
  float D = 1. / (std::numbers::pi * alphax * alphay * (h_alpha_denominator * h_alpha_denominator));
  // get shading, Smith model
  float G_in = G_w(dir_in, alphax, alphay, normal_frame);

  if (reflect) {
    return (F * D * G_in) / (4.f * std::abs(glm::dot(normal_frame.w, dir_in)));
  } else {
    float h_dot_out = glm::dot(half_vec, dir_out);
    float sqrt_denom = h_dot_in + eta * h_dot_out;
    float dh_dout = eta * eta * h_dot_out / (sqrt_denom * sqrt_denom);

    return (1.f - F) * D * G_in * fabs(dh_dout * h_dot_in / glm::dot(normal_frame.w, dir_in));
  }
}

inline std::optional<ScatterInfo> sample_disney_rough_glass(const glm::vec3& dir_in,
                                                            const HitInfo& hit, float mat_eta,
                                                            float anisotropic, float roughness,
                                                            ONB normal_frame,
                                                            pcg32_random_t& pcg_rng) {
  // flip eta when going out of the surface
  float in_geo_dot = glm::dot(dir_in, hit.hit_n_g);
  float eta = in_geo_dot >= 0 ? mat_eta : 1.f / mat_eta;

  constexpr float alpha_min = 0.0001;
  float aspect = std::sqrt(1.f - 0.9f * anisotropic);
  // Clamp roughness to avoid numerical issues.
  roughness = std::clamp(roughness, 0.01f, 1.f);

  float roughness_square = roughness * roughness;

  const float alphax = std::max(alpha_min, roughness_square / aspect);
  const float alphay = std::max(alpha_min, roughness_square * aspect);

  glm::vec3 local_dir_in = project_onto_onb(normal_frame, dir_in);
  glm::vec3 local_micro_normal
      = anisotropic_sample_visible_normals(local_dir_in, alphax, alphay, pcg_rng);

  glm::vec3 half_vec = xform_with_onb(normal_frame, local_micro_normal);

  // Now we need to decide whether to reflect or refract.
  // We do this using the Fresnel term.
  float h_dot_in = glm::dot(half_vec, dir_in);
  float F = fresnel_dielectric(h_dot_in, eta);

  float rand = rand_float(pcg_rng);

  if (rand <= F) {
    // Reflection
    glm::vec3 reflected = glm::normalize(-dir_in + 2 * glm::dot(dir_in, half_vec) * half_vec);
    // set eta to 0 since we are not transmitting
    if (glm::dot(reflected, hit.hit_n_g) * glm::dot(dir_in, hit.hit_n_g) < 0) {
      return std::nullopt;
    } else {
      return ScatterInfo{reflected, 0.f};
    }
  } else {
    // Refraction
    // https://en.wikipedia.org/wiki/Snell%27s_law#Vector_form
    // (note that here eta is eta2 / eta1, and l = -dir_in)
    float h_dot_out_sq = 1 - (1 - h_dot_in * h_dot_in) / (eta * eta);
    if (h_dot_out_sq <= 0) {
      // Total internal reflection
      // This shouldn't really happen, as F will be 1 in this case.
      return std::nullopt;
    }
    // flip half_vector if needed
    if (h_dot_in < 0) {
      half_vec = -half_vec;
    }
    float h_dot_out = sqrt(h_dot_out_sq);
    glm::vec3 refracted = -dir_in / eta + (std::abs(h_dot_in) / eta - h_dot_out) * half_vec;
    if (glm::dot(refracted, hit.hit_n_g) * glm::dot(dir_in, hit.hit_n_g) >= 0) {
      return std::nullopt;
    } else {
      // to avoid getting Nan in pdf when performing refraction
      glm::vec3 generalized_h = glm::normalize(dir_in + refracted * eta);
      float g_h_dot_in = glm::dot(generalized_h, dir_in);

      if ((1 - (1 - g_h_dot_in * g_h_dot_in) / (eta * eta)) < 0) {
        return std::nullopt;
      } else {
        return ScatterInfo{refracted, eta};
      }
    }
  }
}

inline std::pair<glm::vec3, float> eval_pdf_disney_rough_glass(
    const glm::vec3& dir_in, const glm::vec3& dir_out, const HitInfo& hit,
    const glm::vec3& base_col, float mat_eta, glm::vec3 half_vec, ONB normal_frame, const float G,
    const float G_in, const float alphax, const float alphay) {
  glm::vec3 eval = glm::vec3(0.f);
  float pdf = 0.f;

  float in_geo_dot = glm::dot(dir_in, hit.hit_n_g);

  bool reflect = (in_geo_dot * glm::dot(hit.hit_n_g, dir_out)) >= 0;

  // flip eta when going out of the surface
  float eta = in_geo_dot >= 0 ? mat_eta : 1.f / mat_eta;

  if (!reflect) {
    // "Generalized half-vector" from Walter et al.
    half_vec = normalize(dir_in + dir_out * eta);
  }

  float h_dot_in = glm::dot(half_vec, dir_in);
  float F = fresnel_dielectric(h_dot_in, eta);

  const glm::vec3 local_H = project_onto_onb(normal_frame, half_vec);
  float h_alpha_denominator = (local_H.x * local_H.x) / (alphax * alphax)
                              + (local_H.y * local_H.y) / (alphay * alphay)
                              + (local_H.z * local_H.z);
  float D = 1. / (std::numbers::pi * alphax * alphay * (h_alpha_denominator * h_alpha_denominator));

  float normal_in_dot = glm::dot(normal_frame.w, dir_in);

  if (reflect) {
    eval = base_col * (F * D * G) / (4.f * std::abs(normal_in_dot));
    pdf = (F * D * G_in) / (4.f * std::abs(normal_in_dot));
  } else {
    float eta_factor = 1.f / (eta * eta);
    float h_dot_out = glm::dot(half_vec, dir_out);
    float sqrt_denom = h_dot_in + eta * h_dot_out;

    eval = glm::vec3(std::sqrt(base_col.x), std::sqrt(base_col.y), std::sqrt(base_col.z))
           * (eta_factor * (1 - F) * D * G * eta * eta * std::abs(h_dot_out * h_dot_in))
           / (std::abs(normal_in_dot) * sqrt_denom * sqrt_denom);

    float dh_dout = eta * eta * h_dot_out / (sqrt_denom * sqrt_denom);
    pdf = (1.f - F) * D * G_in * std::abs(dh_dout * h_dot_in / normal_in_dot);
  }

  return std::make_pair(eval, pdf);
}