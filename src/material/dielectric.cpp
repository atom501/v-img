#include <material/dielectric.h>
#include <rng/sampling.h>

// return reflected direction. Assumption is that hit_n is normalized
static inline glm::vec3 reflect_dir(const glm::vec3& wi, const glm::vec3& hit_p,
                                    const glm::vec3& hit_n) {
  return wi - (2.f * glm::dot(wi, hit_n) * hit_n);
}

static inline float schlick_apprx(const float cosine, const float in_ior, const float out_ior) {
  auto r0 = (in_ior - out_ior) / (in_ior + out_ior);
  r0 = r0 * r0;
  return r0 + (1.f - r0) * std::powf((1.f - cosine), 5.f);
}

// return refracted direction. Assumption is that hit_n is normalized
static inline glm::vec3 refract_dir(const glm::vec3& wi, const glm::vec3& hit_p,
                                    const glm::vec3& hit_n, const float iIndex_over_oIndex,
                                    const float cos_thetaI, const float sin_thetaT_square) {
  const auto normal_mul = (iIndex_over_oIndex * cos_thetaI) - std::sqrtf(1.f - sin_thetaT_square);

  return (iIndex_over_oIndex * wi) + (normal_mul * hit_n);
}

// how ray will scatter after hitting dielectric material. Source: Reflections and Refractions in
// Ray Tracing by Bram de Greve
std::optional<ScatterInfo> Dielectric::sample_mat(const glm::vec3& wi, const HitInfo& hit,
                                                  pcg32_random_t& pcg_rng) const {
  glm::vec3 wo;  // ray direction after hit

  const float cos_thetaI = -1.f * (glm::dot(wi, hit.hit_n));
  float randf = rand_float(pcg_rng);

  // from air to dielectric
  if (hit.front_face) {
    const float schlick = schlick_apprx(cos_thetaI, 1.0f, ior);

    if (schlick > randf) {
      wo = reflect_dir(wi, hit.hit_p, hit.hit_n);
    } else {
      const auto iIndex_over_oIndex = 1.f / ior;
      const auto sin_thetaT_square
          = (iIndex_over_oIndex * iIndex_over_oIndex) * (1.f - (cos_thetaI * cos_thetaI));
      wo = refract_dir(wi, hit.hit_p, hit.hit_n, iIndex_over_oIndex, cos_thetaI, sin_thetaT_square);
    }
  } else {
    // dielectric to air
    const auto iIndex_over_oIndex = ior;
    const auto sin_thetaT_square
        = (iIndex_over_oIndex * iIndex_over_oIndex) * (1.f - (cos_thetaI * cos_thetaI));

    // total internal reflection or reflection by schlick approx
    if ((sin_thetaT_square > 1.f)
        || (schlick_apprx(std::sqrtf(1.f - sin_thetaT_square), ior, 1.f) > randf)) {
      wo = reflect_dir(wi, hit.hit_p, hit.hit_n);
    } else {
      // refracted
      wo = refract_dir(wi, hit.hit_p, hit.hit_n, iIndex_over_oIndex, cos_thetaI, sin_thetaT_square);
    }
  }

  ScatterInfo ret_inf = {glm::vec3(1.f), wo};
  return std::make_optional(std::move(ret_inf));
}

// has no color of its own
glm::vec3 Dielectric::eval(const glm::vec3& wi, const glm::vec3& wo, const HitInfo& hit) const {
  return glm::vec3(1.f);
}

// since ray only goes in one direction
float Dielectric::pdf(const glm::vec3& wi, const glm::vec3& wo, const HitInfo& hit) const {
  return 0.f;
}