#pragma once

#include <geometry/surface.h>
#include <hit_utils.h>

#include <cstdint>

class Mesh;

class Triangle : public Surface {
public:
  Mesh* obj_mesh = nullptr;
  uint32_t tri_index;  // triangle number in Mesh

public:
  Triangle(Mesh* mesh_ptr, uint32_t index, Material* mat_ptr)
      : obj_mesh(mesh_ptr), tri_index(index), Surface(mat_ptr) {}

  ~Triangle() = default;

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
  inline T tri_hit_template(Ray& ray) {
    const auto& tri_vertex_list = obj_mesh->tri_vertex;
    const auto& vertices_list = obj_mesh->vertices;

    glm::vec3 p0 = vertices_list[tri_vertex_list[3 * tri_index]],
              p1 = vertices_list[tri_vertex_list[3 * tri_index + 1]],
              p2 = vertices_list[tri_vertex_list[3 * tri_index + 2]];

    auto edge1 = p1 - p0;
    auto edge2 = p2 - p0;

    auto pvec = glm::cross(ray.dir, edge2);
    float determinant = glm::dot(pvec, edge1);

    // determinant is close to zero (dir is parallel)
    if (fabs(determinant) < 0.000001) {
      if constexpr (std::is_same_v<T, Surface*>) {
        return nullptr;
      } else if constexpr (std::is_same_v<T, std::optional<HitInfo>>) {
        return std::nullopt;
      }
    }

    float invDet = 1 / determinant;

    auto tvec = ray.o - p0;

    float u = glm::dot(tvec, pvec) * invDet;
    if (u < 0 || u > 1) {
      if constexpr (std::is_same_v<T, Surface*>) {
        return nullptr;
      } else if constexpr (std::is_same_v<T, std::optional<HitInfo>>) {
        return std::nullopt;
      }
    }

    auto qvec = glm::cross(tvec, edge1);

    float v = glm::dot(ray.dir, qvec) * invDet;
    if (v < 0 || u + v > 1) {
      if constexpr (std::is_same_v<T, Surface*>) {
        return nullptr;
      } else if constexpr (std::is_same_v<T, std::optional<HitInfo>>) {
        return std::nullopt;
      }
    }

    float t = glm::dot(qvec, edge2) * invDet;

    if (t < ray.minT || t > ray.maxT) {
      if constexpr (std::is_same_v<T, Surface*>) {
        return nullptr;
      } else if constexpr (std::is_same_v<T, std::optional<HitInfo>>) {
        return std::nullopt;
      }
    }

    // change maxT
    ray.maxT = t;

    if constexpr (std::is_same_v<T, Surface*>) {
      return this;
    } else if constexpr (std::is_same_v<T, std::optional<HitInfo>>) {
      const auto& tri_normal_list = obj_mesh->tri_normal;
      const auto& normal_list = obj_mesh->normals;

      glm::vec3 n0 = normal_list[tri_normal_list[3 * tri_index]],
                n1 = normal_list[tri_normal_list[3 * tri_index + 1]],
                n2 = normal_list[tri_normal_list[3 * tri_index + 2]];

      glm::vec3 normal = glm::normalize(u * n0 + v * n1 + (1 - u - v) * n2);

      const glm::vec3 hit_p = u * p0 + v * p1 + (1 - u - v) * p2;
      const bool front_face = glm::dot(ray.dir, normal) < 0 ? true : false;
      const glm::vec3 hit_n = front_face ? normal : -normal;

      HitInfo hit = {mat, this, hit_p, hit_n, glm::vec2(0.f, 0.f), front_face};

      return std::make_optional(std::move(hit));
    }
  }
};