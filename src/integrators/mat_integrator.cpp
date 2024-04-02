#include <integrators.h>

// returns color produced by a ray from the camera. color value is [0,1]
glm::vec3 material_integrator(Ray& input_ray, const BVH& bvh,
                              const std::vector<std::unique_ptr<Surface>>& prims,
                              const GroupOfEmitters& lights, pcg32_random_t& hash_state,
                              uint32_t depth) {
  auto test_ray = input_ray;
  glm::vec3 throughput = glm::vec3(1.0f);
  constexpr uint32_t roulette_threshold = 5;

  for (size_t d = 0; d <= depth; d++) {
    // perform scene-ray hit test
    std::optional<HitInfo> hit = bvh.hit(test_ray, prims);

    // if ray hits the scene
    if (hit.has_value()) {
      glm::vec3 emitted_col = hit.value().mat->emitted(test_ray, hit.value());
      // get information on scattered ray from material
      float rand_x = rand_float(hash_state);
      float rand_y = rand_float(hash_state);
      float rand_z = rand_float(hash_state);

      std::optional<ScatterInfo> scattered_ray
          = hit.value().mat->sample_mat(test_ray.dir, hit.value(), rand_x, rand_y, rand_z);

      // get color from material (sample the material)
      if (scattered_ray.has_value()) {
        // change in throughput
        throughput
            *= emitted_col
               + (hit.value().mat->eval(test_ray.dir, scattered_ray.value().wo, hit.value())
                  / hit.value().mat->pdf(test_ray.dir, scattered_ray.value().wo, hit.value()));

        // perform russian roulette
        if (d > roulette_threshold) {
          // random [0,1) float
          float rand_float = static_cast<float>(pcg32_random_r(&hash_state))
                             / std::numeric_limits<uint32_t>::max();

          float max_val = std::max(std::max(throughput.x, throughput.y), throughput.z);

          if (rand_float > max_val) {
            break;
          }
          throughput /= max_val;
        }

        // update the ray
        test_ray = Ray(hit.value().hit_p, scattered_ray.value().wo);
      } else {
        return throughput * emitted_col;
      }
    } else {
      // TODO set background color
      return glm::vec3(0.0f);
    }
  }

  // return when depth limit exceeded
  return glm::vec3(0.0f);
}