#include <geometry/sphere.h>

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
  float radius_squared = radius * radius;

  glm::vec3 f = r.o - center;
  float a = glm::dot(r.dir, r.dir);
  float b_prime = glm::dot(-1.0f * f, r.dir);
  float c = glm::dot(f, f) - radius_squared;

  glm::vec3 temp = f + (b_prime / a) * r.dir;
  float discriminant = radius_squared - (glm::dot(temp, temp));

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

  hit.t = t0;
  hit.color = color;
  hit.hit_p = r.o + r.dir * t0;
  hit.hit_n = glm::normalize(hit.hit_p - center);
  hit.mat = mat;

  return std::make_optional(std::move(hit));
}