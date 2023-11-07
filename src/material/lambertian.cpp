#include <material/lambertian.h>

std::optional<ScatterInfo> Lambertian::sample_mat(const glm::vec3& wi, const HitInfo& hit,
                                                  const float& rand1, const float& rand2,
                                                  float rand3) const {
  // transform dir according to hit.sn
  ONB onb = init_onb(hit.hit_n);

  glm::vec3 dir = xform_with_onb(onb, sample_hemisphere_cosine(rand1, rand2));

  if (glm::dot(dir, hit.hit_n) > 0) {
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
  return albedo * static_cast<float>(std::max(0.0f, glm::dot(wo, hit.hit_n)) / std::numbers::pi);
}

float Lambertian::pdf(const glm::vec3& wi, const glm::vec3& wo, const HitInfo& hit) const {
  return static_cast<float>(std::max(0.0f, glm::dot(wo, hit.hit_n)) / std::numbers::pi);
}
