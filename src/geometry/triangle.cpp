#include <geometry/mesh.h>
#include <geometry/triangle.h>
#include <rng/sampling.h>

#include <glm/gtx/norm.hpp>

std::optional<ForHitInfo> Triangle::hit_surface(Ray& ray) {
  return tri_hit_template<std::optional<ForHitInfo>>(ray);
}

bool Triangle::hit_check(Ray& ray) { return tri_hit_template<bool>(ray); }

HitInfo Triangle::hit_info(const Ray& r, const ForHitInfo& pre_calc) {
  const auto& tri_indices = obj_mesh->indices[tri_index];
  const auto& vertices_list = obj_mesh->vertices;

  glm::vec3 p0 = vertices_list[tri_indices[0]], p1 = vertices_list[tri_indices[1]],
            p2 = vertices_list[tri_indices[2]];

  auto edge1 = p1 - p0;
  auto edge2 = p2 - p0;

  float u = pre_calc.e0 * pre_calc.invDet, v = pre_calc.e1 * pre_calc.invDet,
        w = pre_calc.e2 * pre_calc.invDet;

  const glm::vec3 tri_normal = glm::normalize(glm::cross(edge1, edge2));

  glm::vec3 n0, n1, n2;
  glm::vec3 shading_normal;

  const auto& normal_list = obj_mesh->normals;
  if (obj_mesh->normals.size() > 0) {
    n0 = normal_list[tri_indices[0]], n1 = normal_list[tri_indices[1]],
    n2 = normal_list[tri_indices[2]];

    shading_normal = glm::normalize(u * n0 + v * n1 + w * n2);
  } else {
    n0 = tri_normal, n1 = tri_normal, n2 = tri_normal;
    shading_normal = tri_normal;
  }
  const glm::vec3 hit_p = u * p0 + v * p1 + w * p2;

  const auto& texcoords_list = obj_mesh->texcoords;
  const auto& normalcoords_list = obj_mesh->normal_coords;

  glm::vec2 uv = glm::vec2(u, v);
  glm::vec2 diff_uvs[3];
  glm::vec2 uv0 = glm::vec2(0, 0), uv1 = glm::vec2(1, 0), uv2 = glm::vec2(1, 1);
  if (obj_mesh->texcoords.size() > 0) {
    uv0 = texcoords_list[tri_indices[0]], uv1 = texcoords_list[tri_indices[1]],
    uv2 = texcoords_list[tri_indices[2]];

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

  // use normal map if present
  Material* mat = obj_mesh->mat;
  if (mat->normal_map) {
    glm::vec2 n_uv = glm::vec2(u, v);
    glm::vec2 n_uv0 = glm::vec2(0, 0), n_uv1 = glm::vec2(1, 0), n_uv2 = glm::vec2(1, 1);
    if (normalcoords_list.size() > 0) {
      n_uv0 = normalcoords_list[tri_indices[0]], n_uv1 = normalcoords_list[tri_indices[1]],
      n_uv2 = normalcoords_list[tri_indices[2]];

      n_uv = u * n_uv0 + v * n_uv1 + w * n_uv2;
    }

    // get normal and transform using shading normal
    glm::vec3 n_tangent_space = mat->normal_map->get_normal(n_uv);
    ONB onb_n_map = init_onb(shading_normal);

    glm::vec3 local_space_normal = xform_with_onb(onb_n_map, n_tangent_space);

    // set the new shading_normal, dpdu, dpdv
    float ulen = glm::length(dpdu), vlen = glm::length(dpdv);

    dpdu = glm::normalize(GramSchmidt(dpdu, local_space_normal)) * ulen;
    dpdv = glm::normalize(glm::cross(local_space_normal, dpdu)) * vlen;
    shading_normal = local_space_normal;
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

  float uv_area = std::abs((uv1.x - uv0.x) * (uv2.y - uv0.y) - (uv2.x - uv0.x) * (uv1.y - uv0.y));

  return {obj_mesh->mat,
          this,
          hit_p,
          shading_normal,
          tri_normal,
          uv,
          ONB{tangent, bitangent, shading_normal},
          twice_tri_area,
          uv_area,
          mean_curvature};
}

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
