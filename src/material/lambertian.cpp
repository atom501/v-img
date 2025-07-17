#include <material/lambertian.h>

#include <numbers>

std::optional<ScatterInfo> Lambertian::sample_mat(const glm::vec3& wi, const HitInfo& hit,
                                                  pcg32_random_t& pcg_rng) const {
  float rand1 = rand_float(pcg_rng);
  float rand2 = rand_float(pcg_rng);

  // transform dir according to hit.sn
  ONB onb = init_onb(hit.hit_n);

  // dir will be unit vector. point on hemisphere is rotated here (TODO confirm)
  glm::vec3 dir = xform_with_onb(onb, sample_hemisphere_cosine(rand1, rand2));

  if (hit.front_face) {
    ScatterInfo ret_inf = {Lambertian::tex->col_at_uv(hit.uv), dir};
    return std::make_optional(std::move(ret_inf));
  } else {
    return std::nullopt;
  }
}

/*
 * wi: ray going towards material
 * wo: ray going away from material
 * hit: hit information about ray object intersection point
 */
glm::vec3 Lambertian::eval(const glm::vec3& wi, const glm::vec3& wo, const HitInfo& hit) const {
  return Lambertian::tex->col_at_uv(hit.uv)
         * static_cast<float>(std::max(0.0f, glm::dot(wo, hit.hit_n)) / std::numbers::pi);
}

float Lambertian::pdf(const glm::vec3& wi, const glm::vec3& wo, const HitInfo& hit) const {
  return static_cast<float>(std::max(0.0f, glm::dot(wo, hit.hit_n)) / std::numbers::pi);
}

glm::vec3 Lambertian::eval_div_pdf(const glm::vec3& wi, const glm::vec3& wo,
                                   const HitInfo& hit) const {
  return Lambertian::tex->col_at_uv(hit.uv);
}

std::pair<glm::vec3, float> Lambertian::eval_pdf_pair(const glm::vec3& wi, const glm::vec3& wo,
                                                      const HitInfo& hit) const {
  float dot_product
      = static_cast<float>(std::max(0.0f, glm::dot(wo, hit.hit_n)) / std::numbers::pi);

  return std::make_pair(Lambertian::tex->col_at_uv(hit.uv) * dot_product, dot_product);
}
