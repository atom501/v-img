#include <integrators.h>

glm::vec3 mis_integrator(Ray& input_ray, const BVH& bvh,
                         const std::vector<std::unique_ptr<Surface>>& prims,
                         const GroupOfEmitters& lights, pcg32_random_t& hash_state,
                         uint32_t depth) {
  Ray test_ray = input_ray;
  float light_pdf, mat_pdf;
  glm::vec3 bounce_result = glm::vec3(0.0f);
  glm::vec3 throughput = glm::vec3(1.0f);
  size_t d = 0;

  constexpr uint32_t roulette_threshold = 5;

  // perform initial hit test
  std::optional<HitInfo> hit = bvh.hit(test_ray, prims);

  if (!hit.has_value()) {
    // scene missed
    return glm::vec3(0.0f);
  } else if (hit.value().mat->is_emissive()) {
    // if first hit is emissive
    return hit.value().mat->emitted(test_ray, hit.value());
  }

  // If first hit then get color using normal
  for (d = 0; d <= depth; d++) {
    float rand1
        = static_cast<float>(pcg32_random_r(&hash_state)) / std::numeric_limits<uint32_t>::max();
    float rand2
        = static_cast<float>(pcg32_random_r(&hash_state)) / std::numeric_limits<uint32_t>::max();

    // light sampling
    EmitterInfo l_sample_info;
    auto light_col = lights.sample(hit.value().hit_p, l_sample_info, rand1, rand2);

    std::optional<HitInfo> l_visibility_check
        = bvh.hit(Ray(hit.value().hit_p, l_sample_info.wi), prims);

    // if light visible from point
    if (l_visibility_check.has_value() && l_visibility_check.value().obj == l_sample_info.hit.obj) {
      mat_pdf = hit.value().mat->pdf(test_ray.dir, l_sample_info.wi, hit.value());
      float mis_weight = l_sample_info.pdf / (l_sample_info.pdf + mat_pdf);
      bounce_result += throughput
                       * hit.value().mat->eval(test_ray.dir, l_sample_info.wi, hit.value())
                       * mis_weight * light_col / l_sample_info.pdf;
    }

    rand1 = static_cast<float>(pcg32_random_r(&hash_state)) / std::numeric_limits<uint32_t>::max();
    rand2 = static_cast<float>(pcg32_random_r(&hash_state)) / std::numeric_limits<uint32_t>::max();

    float rand3
        = static_cast<float>(pcg32_random_r(&hash_state)) / std::numeric_limits<uint32_t>::max();

    // material sampling. Info for next bounce
    std::optional<ScatterInfo> scattered_mat
        = hit.value().mat->sample_mat(test_ray.dir, hit.value(), rand1, rand2, rand3);

    if (scattered_mat.has_value()) {
      Ray direct_light_ray = Ray(hit.value().hit_p, scattered_mat.value().wo);
      std::optional<HitInfo> hit_next_bounce = bvh.hit(direct_light_ray, prims);

      mat_pdf = hit.value().mat->pdf(test_ray.dir, scattered_mat.value().wo, hit.value());

      throughput
          *= hit.value().mat->eval(test_ray.dir, scattered_mat.value().wo, hit.value()) / mat_pdf;

      // if next ray hits an object
      if (hit_next_bounce.has_value()) {
        // ray hits a light
        if (hit_next_bounce.value().mat->is_emissive()) {
          light_pdf
              = hit_next_bounce.value().obj->pdf(hit.value().hit_p, hit_next_bounce.value().hit_p,
                                                 scattered_mat.value().wo)
                / lights.num_lights();

          float mis_weight = mat_pdf / (light_pdf + mat_pdf);

          bounce_result
              += throughput * mis_weight
                 * hit_next_bounce.value().mat->emitted(direct_light_ray, hit_next_bounce.value());

          // stop bounce since emit encountered
          return bounce_result;
        } else {
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

          // set information for the next bounce
          hit = hit_next_bounce;
          test_ray = direct_light_ray;
        }
      } else {
        // missed scene
        return bounce_result;
      }
    } else {
      return glm::vec3(0.0f);
    }
  }
  return bounce_result;
}