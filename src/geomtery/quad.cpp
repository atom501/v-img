#include <geometry/quads.h>

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

  // Ray hits the 2D shape; set the rest of the hit record,
  HitInfo hit;
  hit.hit_p = intersection;
  hit.color = color;
  hit.mat = mat;
  hit.t = t;
  hit.front_face = glm::dot(r.dir, normal) < 0;
  hit.hit_n = hit.front_face ? normal : -normal;

  return std::make_optional(std::move(hit));
}

AABB Quad::bounds() const {
  // make a AABB but also add some padding if rect is axis aligned
  AABB bbox = AABB(l_corner, l_corner + u + v);
  constexpr float delta = 0.0001;

  if (std::fabs(bbox.box_min.x - bbox.box_max.x) >= delta) {
    bbox.box_min.x -= delta / 2;
    bbox.box_max.x += delta / 2;
  }

  if (std::fabs(bbox.box_min.y - bbox.box_max.y) >= delta) {
    bbox.box_min.y -= delta / 2;
    bbox.box_max.y += delta / 2;
  }

  if (std::fabs(bbox.box_min.z - bbox.box_max.z) >= delta) {
    bbox.box_min.z -= delta / 2;
    bbox.box_max.z += delta / 2;
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
}

glm::vec3 Quad::get_center() const {
  const glm::vec3 opposite_corner = l_corner + u + v;
  return (opposite_corner + l_corner) / 2.0f;
}
