#pragma once

#include <bvh.h>
#include <geometry/surface.h>
#include <hit_utils.h>
#include <material/material.h>

#include <cstdint>

struct MeshData {
public:
  // list of triangle indices. Each triangle has 3 indices for each point
  std::vector<std::array<uint32_t, 3>> indices;
  std::vector<glm::vec3> vertices;  // coordinates of vertices loaded
  BVH tri_bvh;                      // BHV for this mesh. With vertices in Object space

  // optional attributes. may be empty vectors
  std::vector<glm::vec3> normals;    // normals of vertices loaded
  std::vector<glm::vec2> texcoords;  // uv of vertices loaded

  MeshData(const std::vector<std::array<uint32_t, 3>>& indices,
           const std::vector<glm::vec3>& vertices, const std::vector<glm::vec3>& normals,
           const std::vector<glm::vec2>& texcoords)
      : indices(indices), vertices(vertices), normals(normals), texcoords(texcoords) {
    // init BVH
    std::vector<glm::vec3> centers(indices.size());
    std::vector<AABB> AABBs(indices.size());

    for (uint32_t i = 0; const auto& idx : indices) {
      glm::vec3 p0 = vertices[idx[0]];
      glm::vec3 p1 = vertices[idx[1]];
      glm::vec3 p2 = vertices[idx[2]];

      centers[i] = (p0 + p1 + p2) / 3.0f;
      AABBs[i] = AABB(glm::min(p0, glm::min(p1, p2)), glm::max(p0, glm::max(p1, p2)));
    }

    tri_bvh = BVH::build(AABBs, centers, 16);
  }

  ~MeshData() = default;

private:
  /*
   * watertight ray triangle intersection. Source is pbrt and "Watertight Ray/Triangle Intersection"
   * paper
   */
  template <HitReturnType T> inline T tri_hit_template(Ray& ray, uint32_t tri_num) {
    const auto& tri_indices = indices[tri_num];
    glm::vec3 p0 = vertices[p_indices[0]], p1 = vertices[p_indices[1]], p2 = vertices[p_indices[2]];

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
    float Sz = 1.f / d.z;
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

    float invDet = 1.f / det;
    float t = tScaled * invDet;
    ray.maxT = t;

    if constexpr (std::is_same_v<T, bool>) {
      return true;
    } else if constexpr (std::is_same_v<T, std::optional<HitInfo>>) {
      float u = e0 * invDet, v = e1 * invDet, w = e2 * invDet;
      const glm::vec3 tri_normal = glm::normalize(glm::cross(edge1, edge2));

      glm::vec3 n0, n1, n2;
      glm::vec3 shading_normal;
      if (MeshData::normals.size() > 0) {
        n0 = normal_list[tri_indices[0]], n1 = normal_list[tri_indices[1]],
        n2 = normal_list[tri_indices[2]];

        shading_normal = glm::normalize(u * n0 + v * n1 + w * n2);
      } else {
        n0 = tri_normal, n1 = tri_normal, n2 = tri_normal;
        shading_normal = tri_normal;
      }

      const glm::vec3 hit_p = u * p0 + v * p1 + w * p2;

      glm::vec2 uv = glm::vec2(u, v);
      glm::vec2 diff_uvs[3];
      glm::vec2 uv0 = glm::vec2(0, 0), uv1 = glm::vec2(1, 0), uv2 = glm::vec2(1, 1);
      if (Meshdata::texcoords.size() > 0) {
        uv0 = texcoords[tri_indices[0]], uv1 = texcoords[tri_indices[1]],
        uv2 = texcoords[tri_indices[2]];

        uv = u * uv0 + v * uv1 + w * uv2;
      }

      diff_uvs[0] = uv0;
      diff_uvs[1] = uv1;
      diff_uvs[2] = uv2;

      // calculating surface differential
      glm::vec2 duvds = diff_uvs[2] - diff_uvs[0];
      glm::vec2 duvdt = diff_uvs[2] - diff_uvs[1];

      float det = duvds[0] * duvdt[1] - duvdt[0] * duvds[1];

      float dsdu = 0.f, dtdu = 0.f, dsdv = 0.f, dtdv = 0.f;
      glm::vec3 dpdu, dpdv;
      if (std::abs(det) > 1e-8f && !std::isnan(det)) {
        dsdu = duvdt[1] / det;
        dtdu = -duvds[1] / det;
        dsdv = duvdt[0] / det;
        dtdv = -duvds[0] / det;

        // Now we just need to do the matrix multiplication
        glm::vec3 dpds = p2 - p0;
        glm::vec3 dpdt = p2 - p1;
        dpdu = dpds * dsdu + dpdt * dtdu;
        dpdv = dpds * dsdv + dpdt * dtdv;
      } else {
        // degenerate uvs. Use an arbitrary coordinate system
        std::tie(dpdu, dpdv) = get_axis(shading_normal);
      }

      // dpdu may not be orthogonal to shading normal:
      // subtract the projection of shading_normal onto dpdu to make them orthogonal
      glm::vec3 tangent = normalize(dpdu - shading_normal * dot(shading_normal, dpdu));

      // We want to compute dn/du & dn/dv for mean curvature.
      // This is computed in a similar way to dpdu.
      // dn/duv = dn/dst * dst/duv = dn/dst * (duv/dst)^{-1}
      glm::vec3 dnds = n2 - n0;
      glm::vec3 dndt = n2 - n1;
      glm::vec3 dndu = dnds * dsdu + dndt * dtdu;
      glm::vec3 dndv = dnds * dsdv + dndt * dtdv;
      glm::vec3 bitangent = glm::normalize(glm::cross(shading_normal, tangent));
      float mean_curvature = (glm::dot(dndu, tangent) + glm::dot(dndv, bitangent)) / 2.f;

      float twice_tri_area = glm::length(glm::cross(p1 - p0, p2 - p0));

      float uv_area
          = std::abs((uv1.x - uv0.x) * (uv2.y - uv0.y) - (uv2.x - uv0.x) * (uv1.y - uv0.y));

      // TriMeshObj will give the two pointers
      return HitInfo{nullptr,
                     nullptr,
                     tri_num,
                     hit_p,
                     shading_normal,
                     tri_normal,
                     uv,
                     ONB{tangent, bitangent, shading_normal},
                     twice_tri_area,
                     uv_area,
                     mean_curvature};
    }
  }

  std::optional<HitInfo> hit_surface(Ray& r, uint32_t tri_idx) {
    return MeshData::tri_hit_template<std::optional<HitInfo>>(r, tri_idx);
  }

  bool hit_check(Ray& r, uint32_t tri_idx) { return MeshData::tri_hit_template<bool>(r, tri_idx); }
};

class TriMeshObj : public Emitter {
private:
  MeshData* mesh;
  Material* mat;
  glm::mat4 world_to_obj;
  glm::mat4 obj_to_world;

  AABB world_aabb;         // world space AABB
  glm::vec3 world_center;  // world space center

public:
  TriMeshObj(MeshData* mesh, Material* mat, const glm::mat4& world_to_obj,
             const glm::mat4& obj_to_world)
      : mesh(), mat(mat), world_to_obj(world_to_obj), obj_to_world(obj_to_world) {
    world_aabb = AABB(glm::vec3(+std::numeric_limits<float>::max()),
                      glm::vec3(-std::numeric_limits<float>::max()));

    glm::vec3 centroid = glm::vec3(0.f);
    float volume = 0.f;
    for (uint32_t i = 0; const auto& idx : mesh->indices) {
      glm::vec3 p0 = mesh->vertices[idx[0]];
      glm::vec4 temp_o = obj_to_world * glm::vec4(p0, 1.0f);
      temp_o /= temp_o[3];
      p0 = glm::vec3(temp_o);

      glm::vec3 p1 = mesh->vertices[idx[1]];
      glm::vec4 temp_o = obj_to_world * glm::vec4(p1, 1.0f);
      temp_o /= temp_o[3];
      p1 = glm::vec3(temp_o);

      glm::vec3 p2 = mesh->vertices[idx[2]];
      glm::vec4 temp_o = obj_to_world * glm::vec4(p2, 1.0f);
      temp_o /= temp_o[3];
      p2 = glm::vec3(temp_o);

      // https://stackoverflow.com/questions/66891594/calculate-the-centroid-of-a-3d-mesh-of-triangles
      float signed_tera_vol = glm::dot(p0, glm::cross(p1, p2)) / 6.f;

      centroid += signed_tera_vol * (p0 + p1 + p2) / 4.f;
      volume += signed_tera_vol;

      world_aabb.extend(AABB(glm::min(p0, glm::min(p1, p2)), glm::max(p0, glm::max(p1, p2))));
    }

    world_center = centroid / volume;
  }

  ~TriMeshObj() = default;

  std::optional<HitInfo> hit_surface(Ray& r) {
    // transform ray to object space
    Ray dup_ray = r;
    dup_ray.xform_ray(world_to_obj);

    // use ray for bvh intersection
    std::vector<size_t> thread_stack;
    thread_stack.reserve(64);

    std::optional<HitInfo> hit
        = mesh->tri_bvh.hit<std::optional<HitInfo>, MeshData*>(dup_ray, thread_stack, mesh);

    // transform information back to world space where needed
    // update ray maxT

    if (hit.has_value()) {
      auto& hit_val = hit.value();

      const glm::mat4 normal_xform = glm::transpose(world_to_obj);

      hit_val.mat = mat;
      hit_val.obj = this;

      hit_val.hit_n_g = glm::vec3(normal_xform * glm::vec4(hit_val.hit_n_g, 0.0f));
      hit_val.hit_n_s = glm::vec3(normal_xform * glm::vec4(hit_val.hit_n_s, 0.0f));

      hit_val.
    }

    return hit;
  }
  bool hit_check(Ray& r);

  AABB bounds() const { return world_aabb; };
  glm::vec3 get_center() const { return world_center; };
};
