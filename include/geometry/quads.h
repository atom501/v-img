#pragma once

#include <geometry/emitters.h>
#include <geometry/surface.h>
#include <hit_utils.h>
#include <ray.h>

#include <optional>

#include "glm/vec3.hpp"

inline static bool is_interior(float a, float b) {
  // Given the hit point in plane coordinates, return false if it is outside the primitive

  if ((a < 0) || (1 < a) || (b < 0) || (1 < b)) return false;

  return true;
}

class Quad : public Surface, public Emitter {
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

  Quad(const glm::vec3& l_corner, const glm::vec3& u, const glm::vec3& v, Material* mat_ptr,
       const glm::mat4& to_world)
      : Surface(mat_ptr) {
    // transform quad
    glm::vec4 temp_o = to_world * glm::vec4(l_corner, 1.0f);
    temp_o /= temp_o[3];
    Quad::l_corner = temp_o;

    Quad::u = glm::vec3(to_world * glm::vec4(u, 0.0f));
    Quad::v = glm::vec3(to_world * glm::vec4(v, 0.0f));

    auto n = cross(Quad::u, Quad::v);
    normal = glm::normalize(n);
    D = glm::dot(normal, Quad::l_corner);
    w = n / glm::dot(n, n);
  }

  ~Quad() = default;

  std::optional<HitInfo> hit_surface(Ray& r) override;
  bool hit_check(Ray& r) override;

  AABB bounds() const override;
  glm::vec3 get_center() const override;

  std::pair<glm::vec3, EmitterInfo> sample(const glm::vec3& look_from,
                                           pcg32_random_t& pcg_rng) const override;

  float pdf(const glm::vec3& look_from, const glm::vec3& look_at,
            const glm::vec3& dir) const override;

private:
  template <
      typename T,
      std::enable_if_t<std::is_same_v<T, std::optional<HitInfo>> || std::is_same_v<T, bool>, bool>
      = true>
  inline T quad_hit_template(Ray& r) {
    auto denominator = glm::dot(normal, r.dir);

    // No hit if the ray is parallel to the plane.
    if (std::fabs(denominator) < 1e-8) {
      if constexpr (std::is_same_v<T, bool>) {
        return false;
      } else if constexpr (std::is_same_v<T, std::optional<HitInfo>>) {
        return std::nullopt;
      }
    }

    // Return nullopt if the hit point parameter t is outside the ray interval
    auto t = (D - glm::dot(normal, r.o)) / denominator;

    if (t < r.minT || t > r.maxT) {
      if constexpr (std::is_same_v<T, bool>) {
        return false;
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
      if constexpr (std::is_same_v<T, bool>) {
        return false;
      } else if constexpr (std::is_same_v<T, std::optional<HitInfo>>) {
        return std::nullopt;
      }
    }

    // if hit update the maxT for the ray
    r.maxT = t;

    if constexpr (std::is_same_v<T, bool>) {
      return true;
    } else if constexpr (std::is_same_v<T, std::optional<HitInfo>>) {
      // Ray hits the 2D shape; set the rest of the hit record,
      const glm::vec3 hit_p = intersection;

      HitInfo hit = {mat, this, hit_p, Quad::normal, Quad::normal, glm::vec2(alpha, beta)};

      return std::make_optional(std::move(hit));
    }
  }
};