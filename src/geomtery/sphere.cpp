#include <geometry/sphere.h>
#include <rng/sampling.h>

#include <glm/gtx/norm.hpp>

std::optional<HitInfo> Sphere::hit_surface(Ray& ray) {
  return sphere_hit_template<std::optional<HitInfo>>(ray);
}

bool Sphere::hit_check(Ray& ray) { return sphere_hit_template<bool>(ray); }

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

    emit_info.wi = glm::normalize(vec_from_lf_to_pos);

    hit_p = point_on_sphere;
    shading_normal = point_on_unit_sphere;
    // for point inside the sphere direction vec from point to surface of sphere will always be back
    // face

    const float sphere_sa = 4 * std::numbers::pi * radius * radius;

    emit_info.dist = glm::length(vec_from_lf_to_pos);

    if (glm::length2(emit_info.wi) == 0)
      emit_info.pdf = 0;
    else {
      emit_info.pdf = sphere_sa * glm::length2(vec_from_lf_to_pos)
                      / std::abs(glm::dot(shading_normal, -emit_info.wi));
    }
  } else {
    // if look from point is outside the sphere
    float cos_theta_max = sqrt(1.0f - ((radius * radius) / length2(look_from - center)));
    glm::vec3 dir_lf_to_center = glm::normalize(center - look_from);

    // sample direction

    auto onb = init_onb(dir_lf_to_center);
    auto sample_z_dir = sample_sphere_cap(rand1, rand2, cos_theta_max);
    auto sampler_dir = glm::normalize(xform_with_onb(onb, sample_z_dir));

    emit_info.wi = sampler_dir;
    float dist_lf_to_center = glm::length(center - look_from);
    float dist_lf_to_p = dist_lf_to_center - radius;

    hit_p = look_from + sampler_dir * dist_lf_to_p;
    shading_normal = glm::normalize(hit_p - center);
    // for a point outside the sphere the direction vec will always front face

    emit_info.pdf = 1.0f / (2 * std::numbers::pi * (1.0f - cos_theta_max));
    emit_info.dist = dist_lf_to_p;
  }

  glm::vec3 emit_col = mat->emitted(Ray(look_from, emit_info.wi), shading_normal, hit_p);

  return std::make_pair(emit_col, emit_info);
}

float Sphere::pdf(const glm::vec3& look_from, const glm::vec3& look_at,
                  const glm::vec3& dir) const {
  // if look from point is inside the sphere
  if (glm::length2(look_from - center) <= radius * radius) {
    if (glm::length2(dir) == 0)
      return 0;
    else {
      const auto vec_from_lf_to_pos = look_at - look_from;
      const float sphere_sa = 4 * std::numbers::pi * radius * radius;
      const glm::vec3 surface_norm = glm::normalize(look_at - center);

      return sphere_sa * glm::length2(vec_from_lf_to_pos) / std::abs(glm::dot(surface_norm, -dir));
    }
  } else {
    float cos_theta_max = sqrt(1.0f - ((radius * radius) / length2(look_from - center)));

    return 1.0f / (2 * std::numbers::pi * (1.0f - cos_theta_max));
  }
}
