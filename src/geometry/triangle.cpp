#include <geometry/mesh.h>
#include <geometry/triangle.h>
#include <rng/sampling.h>

#include <glm/gtx/norm.hpp>

std::optional<HitInfo> Triangle::hit_surface(Ray& ray) {
  return tri_hit_template<std::optional<HitInfo>>(ray);
}

bool Triangle::hit_check(Ray& ray) { return tri_hit_template<bool>(ray); }

AABB Triangle::bounds() const {
  const auto& tri_indices = obj_mesh->indices[tri_index];
  const auto& vertices_list = obj_mesh->vertices;

  glm::vec3 p0 = vertices_list[tri_indices[0]], p1 = vertices_list[tri_indices[1]],
            p2 = vertices_list[tri_indices[2]];

  auto tri_min_point = glm::min(p0, glm::min(p1, p2));
  auto tri_max_point = glm::max(p0, glm::max(p1, p2));

  return AABB(tri_min_point, tri_max_point);
}

glm::vec3 Triangle::get_center() const {
  const auto& tri_indices = obj_mesh->indices[tri_index];
  const auto& vertices_list = obj_mesh->vertices;

  glm::vec3 p0 = vertices_list[tri_indices[0]], p1 = vertices_list[tri_indices[1]],
            p2 = vertices_list[tri_indices[2]];

  return (p0 + p1 + p2) / 3.0f;
}

std::pair<glm::vec3, EmitterInfo> Triangle::sample(const glm::vec3& look_from,
                                                   pcg32_random_t& pcg_rng) const {
  const auto& tri_indices = obj_mesh->indices[tri_index];

  const auto& vertices_list = obj_mesh->vertices;

  glm::vec3 p0 = vertices_list[tri_indices[0]], p1 = vertices_list[tri_indices[1]],
            p2 = vertices_list[tri_indices[2]];

  const auto edge1 = p1 - p0;
  const auto edge2 = p2 - p0;
  auto tri_normal = glm::normalize(glm::cross(edge1, edge2));

  glm::vec3 n0, n1, n2;
  const auto& normal_list = obj_mesh->normals;
  if (obj_mesh->normals.size() > 0) {
    n0 = normal_list[tri_indices[0]], n1 = normal_list[tri_indices[1]],
    n2 = normal_list[tri_indices[2]];
  } else {
    n0 = tri_normal, n1 = tri_normal, n2 = tri_normal;
  }

  float rand1 = rand_float(pcg_rng);
  float rand2 = rand_float(pcg_rng);

  float u, v;
  // randomly sample u, v
  if (rand1 < rand2) {
    u = rand1 / 2.f;
    v = rand2 - u;
  } else {
    v = rand2 / 2.f;
    u = rand1 - v;
  }

  float w = 1.f - u - v;

  const glm::vec3 hit_p = p0 * u + p1 * v + p2 * w;
  glm::vec3 hit_n = glm::normalize(u * n0 + v * n1 + w * n2);

  auto dir_vec = hit_p - look_from;
  float dist2 = glm::length2(dir_vec);
  dir_vec = glm::normalize(dir_vec);

  float area = glm::length(glm::cross(edge2, edge1)) / 2.0f;

  float pdf = 1.f / area;

  float cosine = std::abs(glm::dot(hit_n, -dir_vec));
  float G = cosine / dist2;

  EmitterInfo emit_info = {dir_vec, pdf, std::sqrtf(dist2), G};
  glm::vec3 emit_col = obj_mesh->mat->emitted(Ray(look_from, emit_info.wi), hit_n, hit_p);

  return std::make_pair(emit_col, emit_info);
}

float Triangle::surf_pdf(const glm::vec3& look_from, const glm::vec3& look_at,
                         const glm::vec3& dir) const {
  const auto& tri_indices = obj_mesh->indices[tri_index];
  const auto& vertices_list = obj_mesh->vertices;

  glm::vec3 p0 = vertices_list[tri_indices[0]], p1 = vertices_list[tri_indices[1]],
            p2 = vertices_list[tri_indices[2]];

  auto edge1 = p1 - p0;
  auto edge2 = p2 - p0;

  float area = glm::length(glm::cross(edge2, edge1)) / 2.0f;
  return 1.f / area;
}
