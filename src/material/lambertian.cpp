#include <material/lambertian.h>

std::optional<ScatterInfo> Lambertian::sample_mat(const glm::vec3& wi, const HitInfo& hit,
                                                  pcg32_random_t& pcg_rng) const {
  float rand1 = rand_float(pcg_rng);
  float rand2 = rand_float(pcg_rng);

  // transform dir according to hit.sn
  ONB onb = init_onb(hit.hit_n);

  // dir will be unit vector. point on hemisphere is rotated here (TODO confirm)
  glm::vec3 dir = xform_with_onb(onb, sample_hemisphere_cosine(rand1, rand2));

  if (hit.front_face) {
    ScatterInfo ret_inf = {Lambertian::albedo, dir};
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
  return albedo * static_cast<float>(std::max(0.0f, glm::dot(wo, hit.hit_n)) / M_PI);
}

float Lambertian::pdf(const glm::vec3& wi, const glm::vec3& wo, const HitInfo& hit) const {
  return static_cast<float>(std::max(0.0f, glm::dot(wo, hit.hit_n)) / M_PI);
}
