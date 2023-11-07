#include <hit_utils.h>
#include <integrators.h>
#include <rng/lcg_rand.h>

#include <cstdint>
#include <optional>

// returns color produced by a ray from the camera. color value is [0,1]
glm::vec3 normal_integrator(Ray& input_ray, const Sphere& s, uint32_t depth) {
  // perform hit test
  std::optional<HitInfo> hit = s.hit(input_ray);

  // If hit then get color using normal
  if (hit.has_value()) {
    auto normal_col = hit.value().hit_n;
    normal_col = (normal_col + 1.0f) / 2.0f;

    return normal_col;
  } else {
    // Else set gradient
    glm::vec3 unit_dir = glm::normalize(input_ray.dir);
    float a = 0.5 * (unit_dir[1] + 1.0);
    glm::vec3 col = (1.0f - a) * glm::vec3(1.0, 1.0, 1.0) + a * glm::vec3(0.5, 0.7, 1.0);

    return col;
  }
}