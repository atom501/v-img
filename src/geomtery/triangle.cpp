#include <geometry/mesh.h>
#include <geometry/triangle.h>
#include <rng/sampling.h>

#include <glm/gtx/norm.hpp>

std::optional<HitInfo> Triangle::hit_surface(Ray& ray) {
  return tri_hit_template<std::optional<HitInfo>>(ray);
}

Surface* Triangle::hit_check(Ray& ray) { return tri_hit_template<Surface*>(ray); }

AABB Triangle::bounds() const {
  const auto& tri_vertex_list = obj_mesh->tri_vertex;
  const auto& vertices_list = obj_mesh->vertices;

  const auto& tri_normal_list = obj_mesh->tri_normal;
  const auto& normal_list = obj_mesh->normals;

  glm::vec3 p0 = vertices_list[tri_vertex_list[3 * tri_index]],
            p1 = vertices_list[tri_vertex_list[3 * tri_index + 1]],
            p2 = vertices_list[tri_vertex_list[3 * tri_index + 2]];

  auto tri_min_point = glm::min(p0, glm::min(p1, p2));
  auto tri_max_point = glm::max(p0, glm::max(p1, p2));

  return AABB(tri_min_point, tri_max_point);
}

glm::vec3 Triangle::get_center() const {
  const auto& tri_vertex_list = obj_mesh->tri_vertex;
  const auto& vertices_list = obj_mesh->vertices;

  const auto& tri_normal_list = obj_mesh->tri_normal;
  const auto& normal_list = obj_mesh->normals;

  glm::vec3 p0 = vertices_list[tri_vertex_list[3 * tri_index]],
            p1 = vertices_list[tri_vertex_list[3 * tri_index + 1]],
            p2 = vertices_list[tri_vertex_list[3 * tri_index + 2]];

  return (p0 + p1 + p2) / 3.0f;
}

std::pair<glm::vec3, EmitterInfo> Triangle::sample(const glm::vec3& look_from,
                                                   pcg32_random_t& pcg_rng) const {
  const auto& tri_vertex_list = obj_mesh->tri_vertex;
  const auto& vertices_list = obj_mesh->vertices;

  const auto& tri_normal_list = obj_mesh->tri_normal;
  const auto& normal_list = obj_mesh->normals;

  glm::vec3 p0 = vertices_list[tri_vertex_list[3 * tri_index]],
            p1 = vertices_list[tri_vertex_list[3 * tri_index + 1]],
            p2 = vertices_list[tri_vertex_list[3 * tri_index + 2]];

  glm::vec3 n0 = normal_list[tri_normal_list[3 * tri_index]],
            n1 = normal_list[tri_normal_list[3 * tri_index + 1]],
            n2 = normal_list[tri_normal_list[3 * tri_index + 2]];

  const auto edge1 = p1 - p0;
  const auto edge2 = p2 - p0;
  auto tri_normal = glm::cross(edge1, edge2);

  float rand1 = rand_float(pcg_rng);
  float rand2 = rand_float(pcg_rng);

  float u = rand1, v = rand2;

  if (u + v > 1) {
    u = 1 - u;
    v = 1 - v;
  }

  float w = 1 - u - v;

  const glm::vec3 hit_p = p0 * u + p1 * v + p2 * w;
  const glm::vec3 hit_n = glm::normalize(u * n0 + v * n1 + (1 - u - v) * n2);

  auto dir_vec = hit_p - look_from;
  float dist2 = glm::length2(dir_vec);
  dir_vec = glm::normalize(dir_vec);

  const bool front_face = glm::dot(dir_vec, tri_normal) < 0 ? true : false;

  const auto& tri_texcoords_list = obj_mesh->tri_normal;
  const auto& texcoords_list = obj_mesh->texcoords;

  glm::vec2 uv = glm::vec2(u, v);
  if (tri_texcoords_list[3 * tri_index] != -1 && tri_texcoords_list[3 * tri_index + 1] != -1
      && tri_texcoords_list[3 * tri_index + 2] != -1) {
    glm::vec2 uv0 = texcoords_list[tri_texcoords_list[3 * tri_index]],
              uv1 = texcoords_list[tri_texcoords_list[3 * tri_index + 1]],
              uv2 = texcoords_list[tri_texcoords_list[3 * tri_index + 2]];

    uv = u * uv0 + v * uv1 + w * uv2;
  }

  HitInfo hit = {mat, this, hit_p, hit_n, uv, front_face};

  // convert to solid angle measure
  float area = glm::length(glm::cross(edge2, edge1)) / 2.0f;
  float cosine = std::abs(glm::dot(tri_normal, dir_vec));
  float pdf = dist2 / (cosine * area);

  EmitterInfo emit_info = {dir_vec, pdf, std::sqrtf(dist2), this};
  glm::vec3 emit_col = mat->emitted(Ray(look_from, emit_info.wi), hit);

  return std::make_pair(emit_col, emit_info);
}

float Triangle::pdf(const glm::vec3& look_from, const glm::vec3& look_at,
                    const glm::vec3& dir) const {
  const auto& tri_vertex_list = obj_mesh->tri_vertex;
  const auto& vertices_list = obj_mesh->vertices;

  const auto& tri_normal_list = obj_mesh->tri_normal;
  const auto& normal_list = obj_mesh->normals;

  glm::vec3 p0 = vertices_list[tri_vertex_list[3 * tri_index]],
            p1 = vertices_list[tri_vertex_list[3 * tri_index + 1]],
            p2 = vertices_list[tri_vertex_list[3 * tri_index + 2]];

  auto edge1 = p1 - p0;
  auto edge2 = p2 - p0;
  auto tri_normal = glm::cross(edge1, edge2);

  float area = glm::length(glm::cross(p2 - p0, p1 - p0)) / 2.0f;
  const glm::vec3 from_lf_to_p = look_at - look_from;
  const float distance2 = glm::length2(from_lf_to_p);
  float cosine = std::abs(glm::dot(dir, tri_normal) / length(dir));
  return distance2 / (cosine * area);
}
