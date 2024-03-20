#include <geometry/mesh.h>
#include <geometry/triangle.h>

#include <glm/gtx/norm.hpp>

std::optional<HitInfo> Triangle::hit(Ray& ray) const {
  const auto& tri_vertex_list = obj_mesh->tri_vertex;
  const auto& vertices_list = obj_mesh->vertices;

  const auto& tri_normal_list = obj_mesh->tri_normal;
  const auto& normal_list = obj_mesh->normals;

  glm::vec3 p0 = vertices_list[tri_vertex_list[3 * tri_index]],
            p1 = vertices_list[tri_vertex_list[3 * tri_index + 1]],
            p2 = vertices_list[tri_vertex_list[3 * tri_index + 2]];

  // glm::vec3 n0 = normal_list[tri_normal_list[3 * tri_index]],
  //           n1 = normal_list[tri_normal_list[3 * tri_index + 1]],
  //           n2 = normal_list[tri_normal_list[3 * tri_index] + 2];

  auto edge1 = p1 - p0;
  auto edge2 = p2 - p0;

  auto pvec = glm::cross(ray.dir, edge2);
  float determinant = glm::dot(pvec, edge1);

  // determinant is close to zero (dir is parallel)
  if (fabs(determinant) < 0.000001) {
    return std::nullopt;
  }

  float invDet = 1 / determinant;

  auto tvec = ray.o - p0;

  float u = glm::dot(tvec, pvec) * invDet;
  if (u < 0 || u > 1) return std::nullopt;

  auto qvec = glm::cross(tvec, edge1);

  float v = glm::dot(ray.dir, qvec) * invDet;
  if (v < 0 || u + v > 1) return std::nullopt;

  float t = glm::dot(qvec, edge2) * invDet;

  glm::vec3 normal = glm::normalize(glm::cross(edge1, edge2));

  // change maxT
  ray.maxT = t;

  HitInfo hit;

  hit.mat = mat;
  hit.obj = this;
  hit.hit_p = ray.at(t);
  hit.front_face = glm::dot(ray.dir, normal) < 0 ? true : false;
  hit.hit_n = hit.front_face ? normal : -normal;

  return std::make_optional(std::move(hit));
}

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

glm::vec3 Triangle::sample(const glm::vec3& look_from, EmitterInfo& emit_info, float rand1,
                           float rand2) const {
  const auto& tri_vertex_list = obj_mesh->tri_vertex;
  const auto& vertices_list = obj_mesh->vertices;

  const auto& tri_normal_list = obj_mesh->tri_normal;
  const auto& normal_list = obj_mesh->normals;

  glm::vec3 p0 = vertices_list[tri_vertex_list[3 * tri_index]],
            p1 = vertices_list[tri_vertex_list[3 * tri_index + 1]],
            p2 = vertices_list[tri_vertex_list[3 * tri_index + 2]];

  glm::vec3 n0 = normal_list[tri_normal_list[3 * tri_index]],
            n1 = normal_list[tri_normal_list[3 * tri_index + 1]],
            n2 = normal_list[tri_normal_list[3 * tri_index] + 2];

  const auto edge1 = p1 - p0;
  const auto edge2 = p2 - p0;
  auto tri_normal = glm::cross(edge1, edge2);

  float u = rand1, v = rand2;

  if (u + v > 1) {
    u = 1 - u;
    v = 1 - v;
  }

  float w = 1 - u - v;

  HitInfo hit;

  hit.mat = mat;
  hit.obj = this;
  hit.hit_p = p0 * u + p1 * v + p2 * w;
  hit.hit_n = glm::normalize(u * n0 + v * n1 + (1 - u - v) * n2);

  auto dir_vec = hit.hit_p - look_from;
  float dist2 = glm::length2(dir_vec);
  dir_vec = glm::normalize(dir_vec);

  hit.front_face = glm::dot(dir_vec, tri_normal) < 0 ? true : false;

  emit_info.hit = hit;
  emit_info.wi = dir_vec;

  // convert to solid angle measure
  float area = glm::length(glm::cross(edge2, edge1)) / 2.0f;
  float cosine = std::abs(glm::dot(tri_normal, dir_vec));
  emit_info.pdf = dist2 / (cosine * area);

  return mat->emitted(Ray(look_from, emit_info.wi), emit_info.hit);
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
