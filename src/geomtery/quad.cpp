#include <geometry/quads.h>

#include <glm/gtx/norm.hpp>
#include <optional>

inline static bool is_interior(float a, float b) {
  // Given the hit point in plane coordinates, return false if it is outside the primitive

  if ((a < 0) || (1 < a) || (b < 0) || (1 < b)) return false;

  return true;
}

std::optional<HitInfo> Quad::hit(Ray& r) const {
  auto denominator = glm::dot(normal, r.dir);

  // No hit if the ray is parallel to the plane.
  if (std::fabs(denominator) < 1e-8) return std::nullopt;

  // Return nullopt if the hit point parameter t is outside the ray interval
  auto t = (D - glm::dot(normal, r.o)) / denominator;

  if (t < r.minT || t > r.maxT) return std::nullopt;

  // Determine the hit point lies within the planar shape using its plane coordinates
  auto intersection = r.at(t);
  glm::vec3 planar_hitpt_vector = intersection - l_corner;
  auto alpha = glm::dot(w, glm::cross(planar_hitpt_vector, v));
  auto beta = glm::dot(w, glm::cross(u, planar_hitpt_vector));

  if (!is_interior(alpha, beta)) return std::nullopt;

  // if hit update the maxT for the ray
  r.maxT = t;

  // Ray hits the 2D shape; set the rest of the hit record,
  HitInfo hit;
  hit.hit_p = intersection;
  hit.mat = mat;
  hit.obj = this;
  hit.front_face = glm::dot(r.dir, normal) < 0;
  hit.hit_n = hit.front_face ? normal : -normal;

  return std::make_optional(std::move(hit));
}

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

void Quad::transform(const glm::mat4& xform) {
  // transform point
  glm::vec4 temp_o = xform * glm::vec4(l_corner, 1.0f);
  temp_o /= temp_o[3];
  l_corner = temp_o;

  // transform vectors starting from l_corner
  u = glm::vec3(xform * glm::vec4(u, 0.0f));
  v = glm::vec3(xform * glm::vec4(v, 0.0f));

  auto n = cross(u, v);
  normal = glm::normalize(n);
  D = glm::dot(normal, l_corner);
  w = n / glm::dot(n, n);
}

glm::vec3 Quad::get_center() const {
  const glm::vec3 opposite_corner = l_corner + u + v;
  return (opposite_corner + l_corner) / 2.0f;
}

glm::vec3 Quad::sample(const glm::vec3& look_from, EmitterInfo& emit_info, float rand1,
                       float rand2) const {
  // random point on the quad
  emit_info.hit.hit_p = l_corner + u * rand1 + v * rand2;
  emit_info.hit.mat = mat;
  emit_info.hit.obj = this;

  const glm::vec3 from_lf_to_p = emit_info.hit.hit_p - look_from;
  const float distance2 = glm::length2(from_lf_to_p);
  emit_info.wi = glm::normalize(from_lf_to_p);

  emit_info.hit.hit_n = normal;

  if (glm::dot(emit_info.wi, normal) > 0) {
    emit_info.hit.front_face = false;
    emit_info.hit.hit_n *= -1.0f;
  } else
    emit_info.hit.front_face = true;

  const float area = glm::length(glm::cross(u, v));
  const float cosine = std::abs(glm::dot(normal, emit_info.wi));
  emit_info.pdf = distance2 / (cosine * area);

  return mat->emitted(Ray(look_from, emit_info.wi), emit_info.hit);
}

float Quad::pdf(const glm::vec3& look_from, const glm::vec3& look_at, const glm::vec3& dir) const {
  const glm::vec3 from_lf_to_p = look_at - look_from;
  const float distance2 = glm::length2(from_lf_to_p);
  const float area = glm::length(glm::cross(u, v));
  const float cosine = std::abs(glm::dot(normal, dir)) / glm::length(dir);

  return distance2 / (cosine * area);
}
