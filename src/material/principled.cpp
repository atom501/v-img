#include <material/disney_helpers/disney_clearcoat.h>
#include <material/disney_helpers/disney_diffuse.h>
#include <material/disney_helpers/disney_sheen.h>
#include <material/principled.h>

std::optional<ScatterInfo> Principled::sample_mat(const glm::vec3& wi, const HitInfo& hit,
                                                  pcg32_random_t& pcg_rng) const {
  glm::vec3 dir_in = -wi;
  ONB normal_frame;

  if ((glm::dot(hit.hit_n_s, dir_in) * glm::dot(hit.hit_n_g, dir_in)) < 0) {
    normal_frame = init_onb(-hit.hit_n_s);
  } else {
    normal_frame = init_onb(hit.hit_n_s);
  }

  float diffuse_weight = (1.f - metallic) * (1.f - specular_transmission);
  float clearcoat_weight = 0.25f * clearcoat;

  float total_w = diffuse_weight + clearcoat_weight;

  float choose_diff = diffuse_weight / total_w;
  float choose_clearcoat = clearcoat_weight / total_w;

  float rnd = rand_float(pcg_rng);

  if (rnd <= choose_diff) {
    return sample_disney_diffuse(dir_in, hit, normal_frame, pcg_rng);
  } else if (rnd > choose_diff && rnd <= choose_diff + choose_clearcoat) {
    return sample_disney_clearcoat(dir_in, hit, normal_frame, clearcoat_gloss, pcg_rng);
  }

  // should be unreachable
  return std::nullopt;
}
glm::vec3 Principled::eval(const glm::vec3& wi, const glm::vec3& wo, const HitInfo& hit) const {
  glm::vec3 dir_in = -wi;
  ONB normal_frame;

  if ((glm::dot(hit.hit_n_s, dir_in) * glm::dot(hit.hit_n_g, dir_in)) < 0) {
    normal_frame = init_onb(-hit.hit_n_s);
  } else {
    normal_frame = init_onb(hit.hit_n_s);
  }

  glm::vec3 half_vector = glm::normalize(dir_in + wo);

  glm::vec3 eval_sheen
      = eval_disney_sheen(dir_in, wo, hit, base_color, sheen_tint, half_vector, normal_frame);

  glm::vec3 eval_diff = eval_disney_diffuse(dir_in, wo, hit, base_color, subsurface, roughness,
                                            half_vector, normal_frame);

  glm::vec3 eval_clearcoat = eval_disney_clearcoat(dir_in, wo, hit, base_color, clearcoat_gloss,
                                                   half_vector, normal_frame);

  return ((1.f - specular_transmission) * (1.f - metallic) * eval_diff)
         + ((1.f - metallic) * sheen * eval_sheen) + (0.25f * clearcoat * eval_clearcoat);
}
float Principled::pdf(const glm::vec3& wi, const glm::vec3& wo, const HitInfo& hit) const {
  glm::vec3 dir_in = -wi;
  ONB normal_frame;

  if ((glm::dot(hit.hit_n_s, dir_in) * glm::dot(hit.hit_n_g, dir_in)) < 0) {
    normal_frame = init_onb(-hit.hit_n_s);
  } else {
    normal_frame = init_onb(hit.hit_n_s);
  }
  glm::vec3 half_vector = glm::normalize(dir_in + wo);

  float diffuse_weight = (1. - metallic) * (1. - specular_transmission);
  float clearcoat_weight = 0.25f * clearcoat;

  float total_w = diffuse_weight + clearcoat_weight;

  float choose_diff = diffuse_weight / total_w;
  float choose_clearcoat = clearcoat_weight / total_w;

  float diff_pdf = pdf_disney_diffuse(dir_in, wo, hit, normal_frame);
  float clearcoat_pdf = pdf_disney_clearcoat(dir_in, wo, hit, base_color, clearcoat_gloss,
                                             half_vector, normal_frame);

  return choose_diff * diff_pdf + choose_clearcoat * clearcoat_pdf;
}

glm::vec3 Principled::eval_div_pdf(const glm::vec3& wi, const glm::vec3& wo,
                                   const HitInfo& hit) const {
  // TODO optimize
  return eval(wi, wo, hit) / pdf(wi, wo, hit);
}

std::pair<glm::vec3, float> Principled::eval_pdf_pair(const glm::vec3& wi, const glm::vec3& wo,
                                                      const HitInfo& hit) const {
  // TODO optimize
  return std::make_pair<glm::vec3, float>(eval(wi, wo, hit), pdf(wi, wo, hit));
}