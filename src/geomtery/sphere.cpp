#include <geometry/sphere.h>
#include <rng/sampling.h>

#include <glm/gtx/norm.hpp>

static inline void solveQuadratic(const float& discriminant, const float& a, const float& b_prime,
                                  const float& c, float& x0, float& x1) {
  float sign = (b_prime > 0) ? 1.0f : -1.0f;
  auto q = b_prime + sign * (sqrt(a * discriminant));

  if (discriminant == 0)
    x0 = x1 = c / q;
  else {
    x0 = c / q;
    x1 = q / a;
  }
  if (x0 > x1) std::swap(x0, x1);
}

std::optional<HitInfo> Sphere::hit_surface(Ray& ray) {
  return sphere_hit_template<std::optional<HitInfo>>(ray);
}

Surface* Sphere::hit_check(Ray& ray) { return sphere_hit_template<Surface*>(ray); }

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

  HitInfo hit;
  EmitterInfo emit_info;

  // if look from point is inside the sphere
  if (glm::length2(look_from - center) <= radius * radius) {
    auto point_on_unit_sphere = sample_sphere(rand1, rand2);
    auto point_on_sphere = (point_on_unit_sphere * radius) + center;
    auto vec_from_lf_to_pos = point_on_sphere - look_from;

    emit_info.wi = glm::normalize(vec_from_lf_to_pos);

    const glm::vec3 hit_p = point_on_sphere;
    const glm::vec3 hit_n = point_on_unit_sphere * -1.0f;
    // for point inside the sphere direction vec from point to surface of sphere will always be back
    // face

    hit = {mat, this, hit_p, hit_n, glm::vec2(0.f, 0.f), false};

    const float sphere_sa = 4 * M_PI * radius * radius;

    emit_info.dist = glm::length(vec_from_lf_to_pos);

    if (glm::length2(emit_info.wi) == 0)
      emit_info.pdf = 0;
    else {
      emit_info.pdf = sphere_sa * glm::length2(vec_from_lf_to_pos)
                      / std::abs(glm::dot(hit.hit_n, -emit_info.wi));
    }
  } else {
    // if look from point is outside the sphere
    float cos_theta_max = sqrt(1.0f - ((radius * radius) / length2(look_from - center)));
    float sin_theta_max2 = std::sqrt(std::max((float)0.f, 1 - cos_theta_max));
    glm::vec3 dir_lf_to_center = glm::normalize(center - look_from);

    // sample direction

    auto onb = init_onb(dir_lf_to_center);
    auto sample_z_dir = sample_sphere_cap(rand1, rand2, cos_theta_max);
    auto sampler_dir = glm::normalize((onb, sample_z_dir));

    emit_info.wi = sampler_dir;
    float costheta = sample_z_dir.z;
    float sintheta_sq = 1 - (costheta * costheta);

    // source pbrtv3
    if (sin_theta_max2 < 0.00068523f /* sin^2(1.5 deg) */) {
      /* Fall back to a Taylor series expansion for small angles, where
         the standard approach suffers from severe cancellation errors */
      sintheta_sq = sin_theta_max2 * rand2;
      costheta = std::sqrt(1 - sintheta_sq);
    }

    float dist_lf_to_center = glm::length(center - look_from);
    float dist_lf_to_p
        = (dist_lf_to_center * costheta)
          - sqrt((radius * radius) - (dist_lf_to_center * dist_lf_to_center) * sintheta_sq);

    const glm::vec3 hit_p = look_from + sampler_dir * dist_lf_to_p;
    const glm::vec3 hit_n = glm::normalize(hit_p - center);
    // for point inside the sphere direction vec from point to surface of sphere will always be
    // front face
    hit = {mat, this, hit_p, hit_n, glm::vec2(0.f, 0.f), true};

    emit_info.pdf = 1.0f / (2 * M_PI * (1.0f - cos_theta_max));
    emit_info.dist = dist_lf_to_p;
  }

  glm::vec3 emit_col = mat->emitted(Ray(look_from, emit_info.wi), hit);

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
      const float sphere_sa = 4 * M_PI * radius * radius;
      const glm::vec3 surface_norm = glm::normalize(look_at - center);

      return sphere_sa * glm::length2(vec_from_lf_to_pos) / std::abs(glm::dot(surface_norm, -dir));
    }
  } else {
    float cos_theta_max = sqrt(1.0f - ((radius * radius) / length2(look_from - center)));

    return 1.0f / (2 * M_PI * (1.0f - cos_theta_max));
  }
}
