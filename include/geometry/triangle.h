#pragma once

#include <geometry/emitters.h>
#include <geometry/surface.h>
#include <hit_utils.h>

#include <cstdint>
#include <glm/gtx/norm.hpp>

class Mesh;

static inline float difference_of_products(float a, float b, float c, float d) {
  float cd = c * d;
  float differenceOfProducts = std::fma(a, b, -cd);
  return differenceOfProducts;
}

static inline double difference_of_products_double(float a, float b, float c, float d) {
  double cd = c * d;
  double differenceOfProducts = std::fmal(a, b, -cd);
  return differenceOfProducts;
}

static inline int max_componenet_index(const glm::vec3& dir) {
  glm::vec3 abs_val = glm::abs(dir);

  int index = 0;
  float max_val = abs_val.x;

  if (abs_val.y > max_val) {
    index = 1;
    max_val = abs_val.y;
  }

  if (abs_val.z > max_val) {
    index = 2;
    max_val = abs_val.z;
  }

  return index;
}

static inline glm::vec3 permute_points(const glm::vec3& orig_vec, const glm::ivec3& indexes) {
  return glm::vec3(orig_vec[indexes.x], orig_vec[indexes.y], orig_vec[indexes.z]);
}

class Triangle : public Surface, public Emitter {
public:
  Mesh* obj_mesh = nullptr;
  uint32_t tri_index;  // triangle number in Mesh

public:
  Triangle(Mesh* mesh_ptr, uint32_t index, Material* mat_ptr)
      : obj_mesh(mesh_ptr), tri_index(index), Surface(mat_ptr) {}

  ~Triangle() = default;

  std::optional<HitInfo> hit_surface(Ray& r) override;
  bool hit_check(Ray& r) override;

  AABB bounds() const override;
  glm::vec3 get_center() const override;

  std::pair<glm::vec3, EmitterInfo> sample(const glm::vec3& look_from,
                                           pcg32_random_t& pcg_rng) const override;

  float pdf(const glm::vec3& look_from, const glm::vec3& look_at,
            const glm::vec3& dir) const override;

private:
  /*
   * watertight ray triangle intersection. Source is pbrt and "Watertight Ray/Triangle Intersection"
   * paper
   */
  template <
      typename T,
      std::enable_if_t<std::is_same_v<T, std::optional<HitInfo>> || std::is_same_v<T, bool>, bool>
      = true>
  inline T tri_hit_template(Ray& ray) {
    const auto& tri_vertex_list = obj_mesh->tri_vertex;
    const auto& vertices_list = obj_mesh->vertices;

    glm::vec3 p0 = vertices_list[tri_vertex_list[3 * tri_index]],
              p1 = vertices_list[tri_vertex_list[3 * tri_index + 1]],
              p2 = vertices_list[tri_vertex_list[3 * tri_index + 2]];

    auto edge1 = p1 - p0;
    auto edge2 = p2 - p0;

    if (glm::length2(glm::cross(edge2, edge1)) == 0.f) {
      if constexpr (std::is_same_v<T, std::optional<HitInfo>>) {
        return std::nullopt;
      } else if constexpr (std::is_same_v<T, bool>) {
        return false;
      }
    }

    // translate points
    glm::vec3 p0t = p0 - ray.o;
    glm::vec3 p1t = p1 - ray.o;
    glm::vec3 p2t = p2 - ray.o;

    // permute componenets based on ray direction axis with the largest absolute value.
    // make z-axis the one with the largest value
    int kz = max_componenet_index(ray.dir);
    int kx = kz + 1;
    if (kx == 3) kx = 0;
    int ky = kx + 1;
    if (ky == 3) ky = 0;

    glm::ivec3 indexes = glm::ivec3(kx, ky, kz);
    glm::vec3 d = permute_points(ray.dir, indexes);
    p0t = permute_points(p0t, indexes);
    p1t = permute_points(p1t, indexes);
    p2t = permute_points(p2t, indexes);

    // apply shear
    float Sx = -d.x / d.z;
    float Sy = -d.y / d.z;
    float Sz = 1 / d.z;
    p0t.x += Sx * p0t.z;
    p0t.y += Sy * p0t.z;
    p1t.x += Sx * p1t.z;
    p1t.y += Sy * p1t.z;
    p2t.x += Sx * p2t.z;
    p2t.y += Sy * p2t.z;

    float e0 = difference_of_products(p1t.x, p2t.y, p1t.y, p2t.x);
    float e1 = difference_of_products(p2t.x, p0t.y, p2t.y, p0t.x);
    float e2 = difference_of_products(p0t.x, p1t.y, p0t.y, p1t.x);

    if (e0 == 0.f || e1 == 0.f || e2 == 0.f) {
      e0 = static_cast<float>(difference_of_products_double(p1t.x, p2t.y, p1t.y, p2t.x));
      e1 = static_cast<float>(difference_of_products_double(p2t.x, p0t.y, p2t.y, p0t.x));
      e2 = static_cast<float>(difference_of_products_double(p0t.x, p1t.y, p0t.y, p1t.x));
    }

    if ((e0 < 0 || e1 < 0 || e2 < 0) && (e0 > 0 || e1 > 0 || e2 > 0)) {
      if constexpr (std::is_same_v<T, std::optional<HitInfo>>) {
        return std::nullopt;
      } else if constexpr (std::is_same_v<T, bool>) {
        return false;
      }
    }

    float det = e0 + e1 + e2;
    if (det == 0) {
      if constexpr (std::is_same_v<T, std::optional<HitInfo>>) {
        return std::nullopt;
      } else if constexpr (std::is_same_v<T, bool>) {
        return false;
      }
    }

    p0t.z *= Sz;
    p1t.z *= Sz;
    p2t.z *= Sz;
    float tScaled = e0 * p0t.z + e1 * p1t.z + e2 * p2t.z;
    if (det < 0 && (tScaled >= 0 || tScaled < ray.maxT * det || tScaled > ray.minT * det)) {
      if constexpr (std::is_same_v<T, std::optional<HitInfo>>) {
        return std::nullopt;
      } else if constexpr (std::is_same_v<T, bool>) {
        return false;
      }
    } else if (det > 0 && (tScaled <= 0 || tScaled > ray.maxT * det || tScaled < ray.minT * det)) {
      if constexpr (std::is_same_v<T, std::optional<HitInfo>>) {
        return std::nullopt;
      } else if constexpr (std::is_same_v<T, bool>) {
        return false;
      }
    }

    float invDet = 1 / det;
    float t = tScaled * invDet;
    ray.maxT = t;

    if constexpr (std::is_same_v<T, bool>) {
      return true;
    } else if constexpr (std::is_same_v<T, std::optional<HitInfo>>) {
      float u = e0 * invDet, v = e1 * invDet, w = e2 * invDet;

      const auto& tri_normal_list = obj_mesh->tri_normal;
      const auto& normal_list = obj_mesh->normals;

      glm::vec3 n0 = normal_list[tri_normal_list[3 * tri_index]],
                n1 = normal_list[tri_normal_list[3 * tri_index + 1]],
                n2 = normal_list[tri_normal_list[3 * tri_index + 2]];

      glm::vec3 normal = glm::normalize(u * n0 + v * n1 + w * n2);

      auto tri_normal = glm::normalize(glm::cross(edge1, edge2));

      const glm::vec3 hit_p = u * p0 + v * p1 + w * p2;
      const bool front_face = glm::dot(ray.dir, normal) < 0 ? true : false;
      const glm::vec3 hit_n = front_face ? normal : -normal;
      tri_normal = front_face ? tri_normal : -tri_normal;

      const auto& tri_texcoords_list = obj_mesh->tri_uv;
      const auto& texcoords_list = obj_mesh->texcoords;

      glm::vec2 uv = glm::vec2(u, v);
      if (tri_texcoords_list[3 * tri_index] != -1 && tri_texcoords_list[3 * tri_index + 1] != -1
          && tri_texcoords_list[3 * tri_index + 2] != -1) {
        glm::vec2 uv0 = texcoords_list[tri_texcoords_list[3 * tri_index]],
                  uv1 = texcoords_list[tri_texcoords_list[3 * tri_index + 1]],
                  uv2 = texcoords_list[tri_texcoords_list[3 * tri_index + 2]];

        uv = u * uv0 + v * uv1 + w * uv2;
      }

      HitInfo hit = {mat, this, hit_p, hit_n, tri_normal, uv, front_face};

      return std::make_optional(std::move(hit));
    }
  }
};