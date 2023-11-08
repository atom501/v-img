#include <hit_utils.h>
#include <integrators.h>
#include <material/material.h>
#include <rng/lcg_rand.h>

#include <cstdint>
#include <optional>

// returns color produced by a ray from the camera. color value is [0,1]
// TODO currently works with same sky as normal integrator. need to cater for lights
glm::vec3 material_integrator(Ray& input_ray, const Sphere& s, pcg32_random_t& hash_state,
                              uint32_t depth) {
  auto test_ray = input_ray;
  glm::vec3 throughput = glm::vec3(1.0f);

  for (size_t i = 0; i < depth; i++) {
    // perform scene-ray hit test
    std::optional<HitInfo> hit = s.hit(test_ray);

    // if ray hits the scene
    if (hit.has_value()) {
      // get information on scattered ray from material
      float rand_x
          = static_cast<float>(pcg32_random_r(&hash_state)) / std::numeric_limits<uint32_t>::max();
      float rand_y
          = static_cast<float>(pcg32_random_r(&hash_state)) / std::numeric_limits<uint32_t>::max();
      float rand_z
          = static_cast<float>(pcg32_random_r(&hash_state)) / std::numeric_limits<uint32_t>::max();

      std::optional<ScatterInfo> scattered_ray
          = hit.value().mat->sample_mat(test_ray.dir, hit.value(), rand_x, rand_y, rand_z);

      // get color from material (sample the material)
      if (scattered_ray.has_value()) {
        // change in throughput
        throughput *= hit.value().mat->eval(test_ray.dir, scattered_ray.value().wo, hit.value())
                      / hit.value().mat->pdf(test_ray.dir, scattered_ray.value().wo, hit.value());

        // update the ray
        test_ray = Ray(hit.value().hit_p, scattered_ray.value().wo);
      } else {
        return glm::vec3(0.0f);
      }
    } else {
      // Else set gradient
      glm::vec3 unit_dir = glm::normalize(input_ray.dir);
      float a = 0.5 * (unit_dir[1] + 1.0);
      glm::vec3 col = (1.0f - a) * glm::vec3(1.0, 1.0, 1.0) + a * glm::vec3(0.5, 0.7, 1.0);

      return throughput * col;
    }
  }

  // return when depth limit exceeded
  return glm::vec3(0.0f);
}