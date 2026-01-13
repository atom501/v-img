#include <geometry/sphere.h>
#include <rng/sampling.h>

#include <glm/gtx/norm.hpp>

std::optional<ForHitInfo> Sphere::hit_surface(Ray& ray) {
  return sphere_hit_template<std::optional<ForHitInfo>>(ray);
}

bool Sphere::hit_check(Ray& ray) { return sphere_hit_template<bool>(ray); }

HitInfo Sphere::hit_info(const Ray& r, const ForHitInfo& pre_calc) {
  const glm::vec3 hit_p = r.o + r.dir * r.maxT;
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

  // fixed value set for Sphere's data needed for texture filtering
  // TODO either remove spheres or change the values
  float mean_curvature = 1.f / Sphere::radius;

  return {mat,
          this,
          hit_p,
          normal,
          normal,
          glm::vec2(u, v),
          ONB{tangent, glm::normalize(glm::cross(normal, tangent)), normal},
          1.f,
          0.000001f,
          mean_curvature};
}

AABB Sphere::bounds() const {
  glm ::vec3 moving_diagonal = glm::vec3(radius, radius, radius);

  glm::vec3 min_box = center - moving_diagonal;
  glm::vec3 max_box = center + moving_diagonal;

  return AABB(min_box, max_box);
}

glm::vec3 Sphere::get_center() const { return center; }

std::pair<glm::vec3, EmitterInfo> Sphere::sample(const glm::vec3& look_from,
                                                 pcg32_random_t& pcg_rng) const {
  float rand1 = rand_float(pcg_rng);
  float rand2 = rand_float(pcg_rng);

  EmitterInfo emit_info;
  glm::vec3 shading_normal;
  glm::vec3 hit_p;

  // if look from point is inside the sphere
  if (glm::length2(look_from - center) <= radius * radius) {
    auto point_on_unit_sphere = sample_sphere(rand1, rand2);
    auto point_on_sphere = (point_on_unit_sphere * radius) + center;
    auto vec_from_lf_to_pos = point_on_sphere - look_from;

    hit_p = point_on_sphere;
    shading_normal = point_on_unit_sphere;
    // for point inside the sphere direction vec from point to surface of sphere will always be back
    // face

    const float sphere_sa = 4.f * std::numbers::pi * radius * radius;

    glm::vec3 dir_to_surf = glm::normalize(vec_from_lf_to_pos);
    float dist2 = glm::length2(vec_from_lf_to_pos);
    float cosine = std::abs(glm::dot(shading_normal, -dir_to_surf));

    float G = cosine / dist2;

    float pdf = 1.f / sphere_sa;
    emit_info = EmitterInfo{dir_to_surf, pdf, std::sqrtf(dist2), G};
  } else {
    // if look from point is outside the sphere
    float cos_theta_max = sqrt(1.0f - ((radius * radius) / length2(look_from - center)));
    glm::vec3 dir_center_to_lf = glm::normalize(look_from - center);

    // sample direction
    auto onb = init_onb(dir_center_to_lf);
    auto sample_z_dir = sample_sphere_cap(rand1, rand2, cos_theta_max);
    auto sampled_point = glm::normalize(xform_with_onb(onb, sample_z_dir)) * radius + center;

    float dist2 = glm::length2(sampled_point - look_from);

    shading_normal = glm::normalize(sampled_point - center);

    glm::vec3 sampled_dir = glm::normalize(sampled_point - look_from);

    float cosine = std::abs(glm::dot(shading_normal, -sampled_dir));
    float G = cosine / dist2;

    // Uniform sampling PDF of a cone
    float pdf_solid_angle = 1.0f / (2.f * std::numbers::pi * (1.0f - cos_theta_max));
    // Convert it back to area measure. Same as other shapes
    float pdf = pdf_solid_angle * G;

    emit_info = EmitterInfo{sampled_dir, pdf, std::sqrtf(dist2), G};
  }

  glm::vec3 emit_col = mat->emitted(Ray(look_from, emit_info.wi), shading_normal, hit_p);

  return std::make_pair(emit_col, emit_info);
}

float Sphere::surf_pdf(const glm::vec3& look_from, const glm::vec3& point_on_light,
                       const glm::vec3& dir) const {
  // if look from point is inside the sphere
  if (glm::length2(look_from - center) <= radius * radius) {
    const float sphere_sa = 4.f * std::numbers::pi * radius * radius;
    return 1.f / sphere_sa;
  } else {
    // if look from point is outside the sphere
    float cos_theta_max = sqrt(1.0f - ((radius * radius) / length2(look_from - center)));

    // Uniform sampling PDF of a cone
    float pdf_solid_angle = 1.0f / (2.f * std::numbers::pi * (1.0f - cos_theta_max));
    // Convert it back to area measure. Same as other shapes
    glm::vec3 shading_normal = glm::normalize(point_on_light - center);
    float cosine = std::abs(glm::dot(shading_normal, -dir));
    float dist2 = glm::length2(point_on_light - look_from);

    return pdf_solid_angle * cosine / dist2;
  }
}
