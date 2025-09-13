#include <geometry/quads.h>
#include <rng/sampling.h>

#include <algorithm>
#include <glm/gtx/norm.hpp>
#include <optional>

std::optional<HitInfo> Quad::hit_surface(Ray& ray) {
  return quad_hit_template<std::optional<HitInfo>>(ray);
}

bool Quad::hit_check(Ray& ray) { return quad_hit_template<bool>(ray); }

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
    bbox.bboxes[0].x -= delta / 2.f;
    bbox.bboxes[1].x += delta / 2.f;
  }

  if (std::fabs(bbox.bboxes[0].y - bbox.bboxes[1].y) >= delta) {
    bbox.bboxes[0].y -= delta / 2.f;
    bbox.bboxes[1].y += delta / 2.f;
  }

  if (std::fabs(bbox.bboxes[0].z - bbox.bboxes[1].z) >= delta) {
    bbox.bboxes[0].z -= delta / 2.f;
    bbox.bboxes[1].z += delta / 2.f;
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
  const float dist2 = glm::length2(from_lf_to_p);
  const glm::vec3 wi = glm::normalize(from_lf_to_p);

  const float area = glm::length(glm::cross(u, v));

  float cosine = std::abs(glm::dot(Quad::normal, -wi));

  const float G = cosine / dist2;

  EmitterInfo emit_info = {wi, 1.f / area, std::sqrtf(dist2), G};
  glm::vec3 emit_col = mat->emitted(Ray(look_from, emit_info.wi), Quad::normal, hit_p);

  return std::make_pair(emit_col, emit_info);
}

float Quad::surf_pdf(const glm::vec3& look_from, const glm::vec3& look_at,
                     const glm::vec3& dir) const {
  const float area = glm::length(glm::cross(u, v));

  return 1.f / area;
}
