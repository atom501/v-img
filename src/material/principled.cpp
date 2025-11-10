#include <material/principled.h>

std::optional<ScatterInfo> Principled::sample_mat(const glm::vec3& wi, const HitInfo& hit,
                                                  pcg32_random_t& pcg_rng) const {
  glm::vec3 dir_in = -wi;
  ONB normal_frame = hit.n_frame;

  if ((glm::dot(hit.hit_n_s, dir_in) * glm::dot(hit.hit_n_g, dir_in)) < 0) {
    normal_frame.u = -normal_frame.u;
    normal_frame.v = -normal_frame.v;
    normal_frame.w = -normal_frame.w;
  }

  if (glm::dot(hit.hit_n_g, dir_in) < 0) {
    return sample_disney_rough_glass(dir_in, hit, eta, anisotropic, roughness, normal_frame,
                                     pcg_rng);
  }

  float diffuse_weight = (1.f - metallic) * (1.f - specular_transmission);
  float clearcoat_weight = 0.25f * clearcoat;
  float metal_weight = (1.f - specular_transmission * (1.f - metallic));
  float glass_weight = (1.f - metallic) * specular_transmission;

  float total_w = diffuse_weight + clearcoat_weight + metal_weight + glass_weight;

  float choose_diff = diffuse_weight / total_w;
  float choose_clearcoat = clearcoat_weight / total_w;
  float choose_metal = metal_weight / total_w;
  float choose_glass = glass_weight / total_w;

  float rnd = rand_float(pcg_rng);

  if (rnd <= choose_diff) {
    return sample_disney_diffuse(dir_in, hit, normal_frame, pcg_rng);
  } else if (rnd > choose_diff && rnd <= (choose_diff + choose_clearcoat)) {
    return sample_disney_clearcoat(dir_in, hit, normal_frame, clearcoat_gloss, pcg_rng);
  } else if (rnd > (choose_diff + choose_clearcoat)
             && rnd <= (choose_diff + choose_clearcoat + choose_metal)) {
    return sample_disney_metal(dir_in, hit, roughness, anisotropic, normal_frame, pcg_rng);
  } else if (rnd > (choose_diff + choose_clearcoat + choose_metal)
             && rnd <= (choose_diff + choose_clearcoat + choose_metal + choose_glass)) {
    return sample_disney_rough_glass(dir_in, hit, eta, anisotropic, roughness, normal_frame,
                                     pcg_rng);
  } else {
    // should be unreachable
    return std::nullopt;
  }
}
glm::vec3 Principled::eval(const glm::vec3& wi, const glm::vec3& wo, const HitInfo& hit,
                           const RayCone& cone) const {
  glm::vec3 dir_in = -wi;
  ONB normal_frame = hit.n_frame;

  if ((glm::dot(hit.hit_n_s, dir_in) * glm::dot(hit.hit_n_g, dir_in)) < 0) {
    normal_frame.u = -normal_frame.u;
    normal_frame.v = -normal_frame.v;
    normal_frame.w = -normal_frame.w;
  }

  glm::vec3 base_color = Principled::tex->col_at_uv(wi, cone, hit);

  glm::vec3 half_vector = glm::normalize(dir_in + wo);
  glm::vec3 eval_glass = eval_disney_rough_glass(dir_in, wo, hit, base_color, eta, anisotropic,
                                                 roughness, half_vector, normal_frame);

  if (glm::dot(hit.hit_n_g, dir_in) < 0) {
    return (1.f - metallic) * specular_transmission * eval_glass;
  }

  glm::vec3 eval_sheen
      = eval_disney_sheen(dir_in, wo, hit, base_color, sheen_tint, half_vector, normal_frame);

  glm::vec3 eval_diff = eval_disney_diffuse(dir_in, wo, hit, base_color, subsurface, roughness,
                                            half_vector, normal_frame);

  glm::vec3 eval_clearcoat = eval_disney_clearcoat(dir_in, wo, hit, base_color, clearcoat_gloss,
                                                   half_vector, normal_frame);

  glm::vec3 eval_metal
      = eval_disney_metal(dir_in, wo, hit, base_color, specular_tint, specular, eta, metallic,
                          roughness, anisotropic, half_vector, normal_frame);

  return ((1.f - specular_transmission) * (1.f - metallic) * eval_diff)
         + ((1.f - metallic) * sheen * eval_sheen) + (0.25f * clearcoat * eval_clearcoat)
         + ((1.f - specular_transmission * (1.f - metallic)) * eval_metal)
         + ((1.f - metallic) * specular_transmission * eval_glass);
}
float Principled::pdf(const glm::vec3& wi, const glm::vec3& wo, const HitInfo& hit) const {
  glm::vec3 dir_in = -wi;
  ONB normal_frame = hit.n_frame;

  if ((glm::dot(hit.hit_n_s, dir_in) * glm::dot(hit.hit_n_g, dir_in)) < 0) {
    normal_frame.u = -normal_frame.u;
    normal_frame.v = -normal_frame.v;
    normal_frame.w = -normal_frame.w;
  }
  glm::vec3 half_vector = glm::normalize(dir_in + wo);

  float glass_pdf = pdf_disney_rough_glass(dir_in, wo, hit, eta, anisotropic, roughness,
                                           half_vector, normal_frame);
  if (glm::dot(hit.hit_n_g, dir_in) < 0) {
    return glass_pdf;
  }

  float diffuse_weight = (1.f - metallic) * (1.f - specular_transmission);
  float clearcoat_weight = 0.25f * clearcoat;
  float metal_weight = (1.f - specular_transmission * (1.f - metallic));
  float glass_weight = (1.f - metallic) * specular_transmission;

  float total_w = diffuse_weight + clearcoat_weight + metal_weight + glass_weight;

  float choose_diff = diffuse_weight / total_w;
  float choose_clearcoat = clearcoat_weight / total_w;
  float choose_metal = metal_weight / total_w;
  float choose_glass = glass_weight / total_w;

  float diff_pdf = pdf_disney_diffuse(dir_in, wo, hit, normal_frame);
  float clearcoat_pdf
      = pdf_disney_clearcoat(dir_in, wo, hit, clearcoat_gloss, half_vector, normal_frame);
  float metal_pdf
      = pdf_disney_metal(dir_in, wo, hit, roughness, anisotropic, half_vector, normal_frame);

  return choose_diff * diff_pdf + choose_clearcoat * clearcoat_pdf + choose_metal * metal_pdf
         + choose_glass * glass_pdf;
}

glm::vec3 Principled::eval_div_pdf(const glm::vec3& wi, const glm::vec3& wo, const HitInfo& hit,
                                   const RayCone& cone) const {
  return eval_pdf<glm::vec3>(wi, wo, hit, cone);
}

std::pair<glm::vec3, float> Principled::eval_pdf_pair(const glm::vec3& wi, const glm::vec3& wo,
                                                      const HitInfo& hit,
                                                      const RayCone& cone) const {
  return eval_pdf<std::pair<glm::vec3, float>>(wi, wo, hit, cone);
}