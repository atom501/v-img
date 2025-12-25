#pragma once

#include <material/disney_helpers/disney_clearcoat.h>
#include <material/disney_helpers/disney_diffuse.h>
#include <material/disney_helpers/disney_glass.h>
#include <material/disney_helpers/disney_metal.h>
#include <material/disney_helpers/disney_sheen.h>
#include <material/material.h>
#include <texture.h>

#include <algorithm>
#include <numbers>

template <typename T>
concept ReturnPair = std::is_same_v<T, std::pair<glm::vec3, float>>;

template <typename T>
concept ReturnEval = std::is_same_v<T, glm::vec3>;

template <typename T>
concept MatReturnType = ReturnEval<T> || ReturnPair<T>;

class Principled : public Material {
private:
  Texture* tex;
  float specular_transmission;
  float metallic;
  float subsurface;
  float specular;
  float roughness;
  float specular_tint;
  float anisotropic;
  float sheen;
  float sheen_tint;
  float clearcoat;
  float clearcoat_gloss;
  float eta;

public:
  Principled(Texture* tex, float specular_transmission, float metallic, float subsurface,
             float specular, float roughness, float specular_tint, float anisotropic, float sheen,
             float sheen_tint, float clearcoat, float clearcoat_gloss, float eta)
      : tex(tex),
        specular_transmission(specular_transmission),
        metallic(metallic),
        subsurface(subsurface),
        specular(specular),
        roughness(roughness),
        specular_tint(specular_tint),
        anisotropic(anisotropic),
        sheen(sheen),
        sheen_tint(sheen_tint),
        clearcoat(clearcoat),
        clearcoat_gloss(clearcoat_gloss),
        eta(eta) {}
  ~Principled() = default;

  std::optional<ScatterInfo> sample_mat(const glm::vec3& wi, const HitInfo& hit,
                                        pcg32_random_t& pcg_rng) const override;
  glm::vec3 eval(const glm::vec3& wi, const glm::vec3& wo, const HitInfo& hit,
                 const RayCone& cone) const override;
  float pdf(const glm::vec3& wi, const glm::vec3& wo, const HitInfo& hit) const override;

  glm::vec3 eval_div_pdf(const glm::vec3& wi, const glm::vec3& wo, const HitInfo& hit,
                         const RayCone& cone) const override;

  std::pair<glm::vec3, float> eval_pdf_pair(const glm::vec3& wi, const glm::vec3& wo,
                                            const HitInfo& hit, const RayCone& cone) const override;

  bool is_delta() const override { return false; }

  // return material eval pdf info as eval/pdf or (eval, pdf)
  template <MatReturnType T> T eval_pdf(const glm::vec3& wi, const glm::vec3& wo,
                                        const HitInfo& hit, const RayCone& cone) const {
    glm::vec3 dir_in = -wi;
    ONB normal_frame = hit.n_frame;

    if ((glm::dot(hit.hit_n_s, dir_in) * glm::dot(hit.hit_n_g, dir_in)) < 0) {
      normal_frame.u = -normal_frame.u;
      normal_frame.v = -normal_frame.v;
      normal_frame.w = -normal_frame.w;
    }

    glm::vec3 base_color = Principled::tex->col_at_ray_hit(wi, cone, hit);

    glm::vec3 half_vector = glm::normalize(dir_in + wo);

    constexpr float alpha_min = 0.0001;
    float aspect = std::sqrt(1.f - 0.9f * anisotropic);
    // Clamp roughness to avoid numerical issues.
    float roughness_clamp = std::clamp(roughness, 0.01f, 1.f);

    float roughness_square = roughness_clamp * roughness_clamp;

    const float alphax = std::max(alpha_min, roughness_square / aspect);
    const float alphay = std::max(alpha_min, roughness_square * aspect);

    float G_in = G_w(dir_in, alphax, alphay, normal_frame);

    float G = G_in * G_w(wo, alphax, alphay, normal_frame);

    auto [eval_glass, pdf_glass] = eval_pdf_disney_rough_glass(
        dir_in, wo, hit, base_color, eta, half_vector, normal_frame, G, G_in, alphax, alphay);

    // only glass if light is under surface
    if (glm::dot(hit.hit_n_g, dir_in) < 0) {
      if constexpr (ReturnPair<T>) {
        return std::make_pair((1.f - metallic) * specular_transmission * eval_glass, pdf_glass);
      } else if constexpr (ReturnEval<T>) {
        return ((1.f - metallic) * specular_transmission * eval_glass) / pdf_glass;
      }
    }

    glm::vec3 eval_sheen
        = eval_disney_sheen(dir_in, wo, hit, base_color, sheen_tint, half_vector, normal_frame);

    auto [eval_diff, pdf_diff] = eval_pdf_disney_diffuse(dir_in, wo, hit, base_color, subsurface,
                                                         roughness, half_vector, normal_frame);

    auto [eval_clearcoat, pdf_clearcoat] = eval_pdf_disney_clearcoat(
        dir_in, wo, hit, base_color, clearcoat_gloss, half_vector, normal_frame);

    auto [eval_metal, pdf_metal]
        = eval_pdf_disney_metal(dir_in, wo, hit, base_color, specular_tint, specular, eta, metallic,
                                half_vector, normal_frame, G, G_in, alphax, alphay);

    glm::vec3 eval_principled = ((1.f - specular_transmission) * (1.f - metallic) * eval_diff)
                                + ((1.f - metallic) * sheen * eval_sheen)
                                + (0.25f * clearcoat * eval_clearcoat)
                                + ((1.f - specular_transmission * (1.f - metallic)) * eval_metal)
                                + ((1.f - metallic) * specular_transmission * eval_glass);

    float diffuse_weight = (1.f - metallic) * (1.f - specular_transmission);
    float clearcoat_weight = 0.25f * clearcoat;
    float metal_weight = (1.f - specular_transmission * (1.f - metallic));
    float glass_weight = (1.f - metallic) * specular_transmission;

    float total_w = diffuse_weight + clearcoat_weight + metal_weight + glass_weight;

    float choose_diff = diffuse_weight / total_w;
    float choose_clearcoat = clearcoat_weight / total_w;
    float choose_metal = metal_weight / total_w;
    float choose_glass = glass_weight / total_w;

    float pdf_principled = choose_diff * pdf_diff + choose_clearcoat * pdf_clearcoat
                           + choose_metal * pdf_metal + choose_glass * pdf_glass;

    if constexpr (ReturnPair<T>) {
      return std::make_pair(eval_principled, pdf_principled);
    } else if constexpr (ReturnEval<T>) {
      return eval_principled / pdf_principled;
    }
  }
};
