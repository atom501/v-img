#pragma once

#include <geometry/surface.h>
#include <hit_utils.h>
#include <ray.h>

#include <optional>

#include "glm/vec3.hpp"

class Sphere : public Surface {
private:
  glm::vec3 center = glm::vec3(0.0f);
  float radius = 1.0f;

public:
  Sphere(const glm::vec3& center, float r, Material* mat_ptr)
      : center(center), radius(r), Surface(mat_ptr) {}

  ~Sphere() = default;

  std::optional<HitInfo> hit_surface(Ray& r) override;
  Surface* hit_check(Ray& r) override;

  AABB bounds() const override;
  glm::vec3 get_center() const override;

  std::pair<glm::vec3, EmitterInfo> sample(const glm::vec3& look_from,
                                           pcg32_random_t& pcg_rng) const override;

  float pdf(const glm::vec3& look_from, const glm::vec3& look_at,
            const glm::vec3& dir) const override;

private:
  // intersection test from ray tracing gems 1, chapter 7
  template <typename T,
            std::enable_if_t<
                std::is_same_v<T, std::optional<HitInfo>> || std::is_same_v<T, Surface*>, bool>
            = true>
  inline T sphere_hit_template(Ray& r) {
    float t0, t1;
    const float radius_squared = radius * radius;

    glm::vec3 f = r.o - center;
    const float a = glm::dot(r.dir, r.dir);
    const float b_prime = glm::dot(-1.0f * f, r.dir);
    const float c = glm::dot(f, f) - radius_squared;

    const glm::vec3 temp = f + (b_prime / a) * r.dir;
    const float discriminant = radius_squared - (glm::dot(temp, temp));

    if (discriminant < 0) {
      if constexpr (std::is_same_v<T, Surface*>) {
        return nullptr;
      } else if constexpr (std::is_same_v<T, std::optional<HitInfo>>) {
        return std::nullopt;
      }
    }

    // get point of ray sphere intersection t0 and t1
    solveQuadratic(discriminant, a, b_prime, c, t0, t1);

    if (t0 < r.minT || t0 > r.maxT) {
      t0 = t1;
      if (t0 < r.minT || t0 > r.maxT) {
        if constexpr (std::is_same_v<T, Surface*>) {
          return nullptr;
        } else if constexpr (std::is_same_v<T, std::optional<HitInfo>>) {
          return std::nullopt;
        }
      }
    }

    // if hit update the maxT for the ray
    r.maxT = t0;

    if constexpr (std::is_same_v<T, Surface*>) {
      return this;
    } else if constexpr (std::is_same_v<T, std::optional<HitInfo>>) {
      const glm::vec3 hit_p = r.o + r.dir * t0;
      const glm::vec3 normal = glm::normalize(hit_p - center);
      const bool front_face = glm::dot(r.dir, normal) < 0;
      const glm::vec3 hit_n = front_face ? normal : -normal;

      HitInfo hit = {mat, this, hit_p, hit_n, front_face};

      return std::make_optional(std::move(hit));
    }
  };
};
