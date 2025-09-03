#pragma once

#include <geometry/emitters.h>
#include <geometry/surface.h>
#include <hit_utils.h>
#include <ray.h>

#include <numbers>
#include <optional>

#include "glm/vec3.hpp"

static inline void solveQuadratic(const float& discriminant, const float& a, const float& b_prime,
                                  const float& c, float& x0, float& x1) {
  float sign = (b_prime > 0) ? 1.0f : -1.0f;
  auto q = b_prime + sign * (sqrt(a * discriminant));

  if (discriminant == 0)
    x0 = x1 = c / q;
  else {
    x0 = c / q;
    x1 = q / a;
  }
  if (x0 > x1) std::swap(x0, x1);
}

class Sphere : public Surface, public Emitter {
private:
  glm::vec3 center = glm::vec3(0.0f);
  float radius = 1.0f;

public:
  Sphere(const glm::vec3& center, float r, Material* mat_ptr)
      : center(center), radius(r), Surface(mat_ptr) {}

  ~Sphere() = default;

  std::optional<HitInfo> hit_surface(Ray& r) override;
  bool hit_check(Ray& r) override;

  AABB bounds() const override;
  glm::vec3 get_center() const override;

  std::pair<glm::vec3, EmitterInfo> sample(const glm::vec3& look_from,
                                           pcg32_random_t& pcg_rng) const override;

  float pdf(const glm::vec3& look_from, const glm::vec3& look_at,
            const glm::vec3& dir) const override;

private:
  // intersection test from ray tracing gems 1, chapter 7
  template <
      typename T,
      std::enable_if_t<std::is_same_v<T, std::optional<HitInfo>> || std::is_same_v<T, bool>, bool>
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
      if constexpr (std::is_same_v<T, bool>) {
        return false;
      } else if constexpr (std::is_same_v<T, std::optional<HitInfo>>) {
        return std::nullopt;
      }
    }

    // get point of ray sphere intersection t0 and t1
    solveQuadratic(discriminant, a, b_prime, c, t0, t1);

    if (t0 < r.minT || t0 > r.maxT) {
      t0 = t1;
      if (t0 < r.minT || t0 > r.maxT) {
        if constexpr (std::is_same_v<T, bool>) {
          return false;
        } else if constexpr (std::is_same_v<T, std::optional<HitInfo>>) {
          return std::nullopt;
        }
      }
    }

    // if hit update the maxT for the ray
    r.maxT = t0;

    if constexpr (std::is_same_v<T, bool>) {
      return true;
    } else if constexpr (std::is_same_v<T, std::optional<HitInfo>>) {
      const glm::vec3 hit_p = r.o + r.dir * t0;
      const glm::vec3 normal = glm::normalize(hit_p - center);

      // calculate uv
      float theta = std::acos(-normal.y);
      float phi = std::atan2(-normal.z, normal.x) + std::numbers::pi;

      float u = phi / (2.f * std::numbers::pi);
      float v = theta / std::numbers::pi;

      glm::vec3 dpdu{-Sphere::radius * normal.y, Sphere::radius * normal.x, 0.f};
      glm::vec3 dpdv{Sphere::radius * std::cos(u) * std::cos(v),
                     Sphere::radius * std::sin(u) * std::cos(v), -Sphere::radius * std::sin(v)};
      // dpdu may not be orthogonal to shading normal:
      // subtract the projection of shading_normal onto dpdu to make them orthogonal
      glm::vec3 tangent = glm::normalize(dpdu - normal * dot(normal, dpdu));

      HitInfo hit = {mat,
                     this,
                     hit_p,
                     normal,
                     normal,
                     glm::vec2(u, v),
                     ONB{tangent, glm::normalize(glm::cross(normal, tangent)), normal}};

      return std::make_optional(std::move(hit));
    }
  }
};
