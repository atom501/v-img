#include <integrators.h>

// returns color produced by a ray from the camera. color value is [0,1]
glm::vec3 normal_integrator(Ray& input_ray, std::vector<size_t>& thread_stack, const BVH& bvh,
                            const std::vector<std::unique_ptr<Surface>>& prims,
                            const GroupOfEmitters& lights, pcg32_random_t& hash_state,
                            uint32_t depth, Background* background) {
  // perform hit test
  std::optional<HitInfo> hit = bvh.hit<std::optional<HitInfo>>(input_ray, thread_stack, prims);

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