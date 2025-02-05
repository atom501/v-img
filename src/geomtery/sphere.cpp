#include <geometry/sphere.h>
#include <rng/sampling.h>

#include <glm/gtx/norm.hpp>

static inline void solveQuadratic(const float &discriminant, const float &a, const float &b_prime,
                                  const float &c, float &x0, float &x1) {
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

// intersection test from ray tracing gems 1, chapter 7
std::optional<HitInfo> Sphere::hit(Ray &r) const {
  float t0, t1;
  const float radius_squared = radius * radius;

  glm::vec3 f = r.o - center;
  const float a = glm::dot(r.dir, r.dir);
  const float b_prime = glm::dot(-1.0f * f, r.dir);
  const float c = glm::dot(f, f) - radius_squared;

  const glm::vec3 temp = f + (b_prime / a) * r.dir;
  const float discriminant = radius_squared - (glm::dot(temp, temp));

  if (discriminant < 0) return std::nullopt;

  // get point of ray sphere intersection t0 and t1
  solveQuadratic(discriminant, a, b_prime, c, t0, t1);

  if (t0 < r.minT || t0 > r.maxT) {
    t0 = t1;
    if (t0 < r.minT || t0 > r.maxT) return std::nullopt;
  }

  // if hit update the maxT for the ray
  r.maxT = t0;

  HitInfo hit;

  hit.hit_p = r.o + r.dir * t0;

  const glm::vec3 normal = glm::normalize(hit.hit_p - center);

  hit.front_face = glm::dot(r.dir, normal) < 0;
  hit.hit_n = hit.front_face ? normal : -normal;
  hit.mat = mat;
  hit.obj = this;

  return std::make_optional(std::move(hit));
}

AABB Sphere::bounds() const {
  glm ::vec3 moving_diagonal = glm::vec3(radius, radius, radius);

  glm::vec3 min_box = center - moving_diagonal;
  glm::vec3 max_box = center + moving_diagonal;

  return AABB(min_box, max_box);
}

glm::vec3 Sphere::get_center() const { return center; }

std::pair<glm::vec3, EmitterInfo> Sphere::sample(const glm::vec3 &look_from,
                                                 pcg32_random_t &pcg_rng) const {
  float rand1 = rand_float(pcg_rng);
  float rand2 = rand_float(pcg_rng);

  EmitterInfo emit_info;

  // if look from point is inside the sphere
  if (glm::length2(look_from - center) <= radius * radius) {
    auto point_on_unit_sphere = sample_sphere(rand1, rand2);
    auto point_on_sphere = (point_on_unit_sphere * radius) + center;
    auto vec_from_lf_to_pos = point_on_sphere - look_from;

    emit_info.wi = glm::normalize(vec_from_lf_to_pos);

    emit_info.hit.hit_p = point_on_sphere;
    emit_info.hit.mat = mat;
    emit_info.hit.obj = this;
    emit_info.hit.hit_n = point_on_unit_sphere * -1.0f;
    // for point inside the sphere direction vec from point to surface of sphere will always be back
    // face
    emit_info.hit.front_face = false;

    const float sphere_sa = 4 * M_PI * radius * radius;

    if (glm::length2(emit_info.wi) == 0)
      emit_info.pdf = 0;
    else {
      emit_info.pdf = sphere_sa * glm::length2(vec_from_lf_to_pos)
                      / std::abs(glm::dot(emit_info.hit.hit_n, -emit_info.wi));
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

    emit_info.hit.hit_p = look_from + sampler_dir * dist_lf_to_p;
    emit_info.hit.hit_n = glm::normalize(emit_info.hit.hit_p - center);
    // for point inside the sphere direction vec from point to surface of sphere will always be
    // front face
    emit_info.hit.front_face = true;
    emit_info.hit.mat = mat;
    emit_info.hit.obj = this;

    emit_info.pdf = 1.0f / (2 * M_PI * (1.0f - cos_theta_max));
  }

  glm::vec3 emit_col = mat->emitted(Ray(look_from, emit_info.wi), emit_info.hit);

  return std::make_pair(emit_col, emit_info);
}

float Sphere::pdf(const glm::vec3 &look_from, const glm::vec3 &look_at,
                  const glm::vec3 &dir) const {
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
