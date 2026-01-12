#pragma once
#include <hit_utils.h>
#include <ray.h>
#include <rng/pcg_rand.h>

#include <glm/vec3.hpp>
#include <glm/vec4.hpp>
#include <optional>

struct HitInfo;

struct ScatterInfo {
  glm::vec3 wo;  // outgoing direction from hit point on material. Always normalized
  float eta;     // get eta for the case with refraction
  bool is_specular;
};

namespace MatConst {
  constexpr float regularize_min = 0.03f;
  constexpr float regularize_max = 0.1f;
  constexpr float roughness_threshold = 0.1f;
}

class Material {
public:
  Material() = default;
  ~Material() = default;

  /* wi: ray direction towards surface
   *  hit: information of point of hit
   */
  virtual std::optional<ScatterInfo> sample_mat(const glm::vec3& wi, const HitInfo& hit,
                                                pcg32_random_t& pcg_rng, bool regularize) const {
    return std::nullopt;
  }

  virtual glm::vec3 eval(const glm::vec3& wi, const glm::vec3& wo, const HitInfo& hit,
                         const RayCone& cone) const {
    return glm::vec3(0.0f);
  }

  virtual float pdf(const glm::vec3& wi, const glm::vec3& wo, const HitInfo& hit) const {
    return 1.0f;
  }

  virtual glm::vec3 eval_div_pdf(const glm::vec3& wi, const glm::vec3& wo, const HitInfo& hit,
                                 const RayCone& cone, bool regularize) const {
    return glm::vec3(0.0f);
  }

  virtual std::pair<glm::vec3, float> eval_pdf_pair(const glm::vec3& wi, const glm::vec3& wo,
                                                    const HitInfo& hit, const RayCone& cone,
                                                    bool regularize) const {
    return std::make_pair(glm::vec3(0.0f), 1.0f);
  }

  // light emitted
  virtual glm::vec3 emitted(const Ray& ray, const glm::vec3& shading_normal,
                            const glm::vec3& hit_p) const {
    return glm::vec3(0.0f, 0.0f, 0.0f);
  }

  virtual bool is_emissive() const { return false; }

  // return true if mat pdf is a delta function
  virtual bool is_delta() const { return true; }
};
