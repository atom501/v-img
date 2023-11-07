#pragma once
#include <hit_utils.h>

#include <glm/vec3.hpp>
#include <glm/vec4.hpp>
#include <optional>

struct HitInfo;

struct ScatterInfo {
  glm::vec3 attenuation;
  glm::vec3 wo;  // outgoing direction from hit point on material
};

class Material {
public:
  Material(){};
  ~Material(){};

  /* wi: ray direction towards surface
   *  hit: information of point of hit
   *  srec: information about the new outgoing ray from surface
   *  rv: 2 random numbers
   *  rv1: 1 random number
   */
  virtual std::optional<ScatterInfo> sample_mat(const glm::vec3& wi, const HitInfo& hit,
                                                const float& rand1, const float& rand2,
                                                float rand3) const {
    return std::nullopt;
  }

  virtual glm::vec3 eval(const glm::vec3& wi, const glm::vec3& wo, const HitInfo& hit) const {
    return glm::vec3(0.0f);
  }

  virtual float pdf(const glm::vec3& wi, const glm::vec3& wo, const HitInfo& hit) const {
    return 1.0f;
  }
};
