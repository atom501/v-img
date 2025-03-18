#pragma once

#include <geometry/surface.h>
#include <hit_utils.h>
#include <ray.h>

#include <optional>

#include "glm/vec3.hpp"

class Quad : public Surface {
private:
  glm::vec3 l_corner = glm::vec3(0.0f);  // lower-left corner of a quad
  glm::vec3 u, v;                        // vectors starting from l_corner
  glm::vec3 normal;
  glm::vec3 w;  // constant for the plane's basis frame
  float D;      // D in eq Ax+By+Cz=D

public:
  Quad(const glm::vec3& l_corner, const glm::vec3& u, const glm::vec3& v, Material* mat_ptr)
      : l_corner(l_corner), u(u), v(v), Surface(mat_ptr) {
    auto n = cross(u, v);
    normal = glm::normalize(n);
    D = glm::dot(normal, l_corner);
    w = n / glm::dot(n, n);
  }

  ~Quad() = default;

  std::optional<HitInfo> hit_surface(Ray& r) override;
  Surface* hit_check(Ray& r) override;

  AABB bounds() const override;
  glm::vec3 get_center() const override;

  std::pair<glm::vec3, EmitterInfo> sample(const glm::vec3& look_from,
                                           pcg32_random_t& pcg_rng) const override;

  float pdf(const glm::vec3& look_from, const glm::vec3& look_at,
            const glm::vec3& dir) const override;

private:
  template <typename T,
            std::enable_if_t<
                std::is_same_v<T, std::optional<HitInfo>> || std::is_same_v<T, Surface*>, bool>
            = true>
  inline T quad_hit_template(Ray& r) {
    auto denominator = glm::dot(normal, r.dir);

    // No hit if the ray is parallel to the plane.
    if (std::fabs(denominator) < 1e-8) {
      if constexpr (std::is_same_v<T, Surface*>) {
        return nullptr;
      } else if constexpr (std::is_same_v<T, std::optional<HitInfo>>) {
        return std::nullopt;
      }
    }

    // Return nullopt if the hit point parameter t is outside the ray interval
    auto t = (D - glm::dot(normal, r.o)) / denominator;

    if (t < r.minT || t > r.maxT) {
      if constexpr (std::is_same_v<T, Surface*>) {
        return nullptr;
      } else if constexpr (std::is_same_v<T, std::optional<HitInfo>>) {
        return std::nullopt;
      }
    }

    // Determine the hit point lies within the planar shape using its plane coordinates
    auto intersection = r.at(t);
    glm::vec3 planar_hitpt_vector = intersection - l_corner;
    auto alpha = glm::dot(w, glm::cross(planar_hitpt_vector, v));
    auto beta = glm::dot(w, glm::cross(u, planar_hitpt_vector));

    if (!is_interior(alpha, beta)) {
      if constexpr (std::is_same_v<T, Surface*>) {
        return nullptr;
      } else if constexpr (std::is_same_v<T, std::optional<HitInfo>>) {
        return std::nullopt;
      }
    }

    // if hit update the maxT for the ray
    r.maxT = t;

    if constexpr (std::is_same_v<T, Surface*>) {
      return this;
    } else if constexpr (std::is_same_v<T, std::optional<HitInfo>>) {
      // Ray hits the 2D shape; set the rest of the hit record,
      const glm::vec3 hit_p = intersection;
      const bool front_face = glm::dot(r.dir, normal) < 0;
      const glm::vec3 hit_n = front_face ? normal : -normal;

      HitInfo hit = {mat, this, hit_p, hit_n, front_face};

      return std::make_optional(std::move(hit));
    }
  };
};