#include <geometry/quads.h>
#include <rng/sampling.h>

#include <algorithm>
#include <glm/gtx/norm.hpp>
#include <optional>

inline static bool is_interior(float a, float b) {
  // Given the hit point in plane coordinates, return false if it is outside the primitive

  if ((a < 0) || (1 < a) || (b < 0) || (1 < b)) return false;

  return true;
}

std::optional<HitInfo> Quad::hit_surface(Ray& ray) {
  return quad_hit_template<std::optional<HitInfo>>(ray);
}

Surface* Quad::hit_check(Ray& ray) { return quad_hit_template<Surface*>(ray); }

// returns AABB. After transform l_corner is moved so better to check all 4 corners
AABB Quad::bounds() const {
  // make a AABB but also add some padding if rect is axis aligned
  const glm::vec3& point1 = l_corner;
  const glm::vec3 point2 = l_corner + u;
  const glm::vec3 point3 = l_corner + v;
  const glm::vec3 point4 = l_corner + v + u;

  glm::vec3 min_b;
  min_b.x = std::min({point1.x, point2.x, point3.x, point4.x});
  min_b.y = std::min({point1.y, point2.y, point3.y, point4.y});
  min_b.z = std::min({point1.z, point2.z, point3.z, point4.z});

  glm::vec3 max_b;
  max_b.x = std::max({point1.x, point2.x, point3.x, point4.x});
  max_b.y = std::max({point1.y, point2.y, point3.y, point4.y});
  max_b.z = std::max({point1.z, point2.z, point3.z, point4.z});

  AABB bbox = AABB(min_b, max_b);

  constexpr float delta = 0.0001;

  if (std::fabs(bbox.bboxes[0].x - bbox.bboxes[1].x) >= delta) {
    bbox.bboxes[0].x -= delta / 2;
    bbox.bboxes[1].x += delta / 2;
  }

  if (std::fabs(bbox.bboxes[0].y - bbox.bboxes[1].y) >= delta) {
    bbox.bboxes[0].y -= delta / 2;
    bbox.bboxes[1].y += delta / 2;
  }

  if (std::fabs(bbox.bboxes[0].z - bbox.bboxes[1].z) >= delta) {
    bbox.bboxes[0].z -= delta / 2;
    bbox.bboxes[1].z += delta / 2;
  }

  return bbox;
}

glm::vec3 Quad::get_center() const {
  const glm::vec3 opposite_corner = l_corner + u + v;
  return (opposite_corner + l_corner) / 2.0f;
}

std::pair<glm::vec3, EmitterInfo> Quad::sample(const glm::vec3& look_from,
                                               pcg32_random_t& pcg_rng) const {
  float rand1 = rand_float(pcg_rng);
  float rand2 = rand_float(pcg_rng);

  // random point on the quad
  const glm::vec3 hit_p = l_corner + u * rand1 + v * rand2;

  const glm::vec3 from_lf_to_p = hit_p - look_from;
  const float distance2 = glm::length2(from_lf_to_p);
  const glm::vec3 wi = glm::normalize(from_lf_to_p);

  glm::vec3 hit_n = normal;
  bool front_face;

  if (glm::dot(wi, normal) > 0) {
    front_face = false;
    hit_n *= -1.0f;
  } else
    front_face = true;

  const float area = glm::length(glm::cross(u, v));
  const float cosine = std::abs(glm::dot(normal, wi));
  const float pdf = distance2 / (cosine * area);

  HitInfo hit = {mat, this, hit_p, hit_n, front_face};

  EmitterInfo emit_info = {wi, pdf, std::sqrtf(distance2), this};

  glm::vec3 emit_col = mat->emitted(Ray(look_from, emit_info.wi), hit);

  return std::make_pair(emit_col, emit_info);
}

float Quad::pdf(const glm::vec3& look_from, const glm::vec3& look_at, const glm::vec3& dir) const {
  const glm::vec3 from_lf_to_p = look_at - look_from;
  const float distance2 = glm::length2(from_lf_to_p);
  const float area = glm::length(glm::cross(u, v));
  const float cosine = std::abs(glm::dot(normal, dir)) / glm::length(dir);

  return distance2 / (cosine * area);
}
