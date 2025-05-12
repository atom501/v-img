#include <integrators.h>

glm::vec3 mis_integrator(Ray& input_ray, std::vector<size_t>& thread_stack, const BVH& bvh,
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
  std::optional<HitInfo> hit = bvh.hit<std::optional<HitInfo>>(test_ray, thread_stack, prims);

  if (!hit.has_value()) {
    // scene missed
    return glm::vec3(0.0f);
  } else if (hit.value().mat->is_emissive()) {
    // if first hit is emissive
    return hit.value().mat->emitted(test_ray, hit.value());
  }

  // If first hit does not miss or hit light
  for (d = 0; d <= depth; d++) {
    // info for next bounce
    std::optional<ScatterInfo> scattered_mat
        = hit.value().mat->sample_mat(test_ray.dir, hit.value(), hash_state);

    if (!scattered_mat.has_value()) {
      return glm::vec3(0.0f);
    }

    float mat_sample_pdf
        = hit.value().mat->pdf(test_ray.dir, scattered_mat.value().wo, hit.value());

    // skip if light sampling handling delta functions with material pdf = 0
    if (mat_sample_pdf != 0) {
      // light sampling
      auto [light_col, l_sample_info] = lights.sample(hit.value().hit_p, hash_state);

      Ray shadow_ray = Ray(hit.value().hit_p, l_sample_info.wi);
      shadow_ray.maxT = l_sample_info.dist + 0.0001f;
      Surface* l_visibility_check = bvh.hit<Surface*>(shadow_ray, thread_stack, prims);

      // if light visible from point
      if (l_visibility_check && l_visibility_check == l_sample_info.obj) {
        const auto [mat_eval, mat_pdf]
            = hit.value().mat->eval_pdf_pair(test_ray.dir, l_sample_info.wi, hit.value());

        float mis_weight = l_sample_info.pdf / (l_sample_info.pdf + mat_pdf);
        bounce_result += throughput * mat_eval * mis_weight * light_col / l_sample_info.pdf;
      }
    }

    // material sampling
    throughput
        *= hit.value().mat->eval_div_pdf(test_ray.dir, scattered_mat.value().wo, hit.value());

    // where light goes after hitting object
    Ray direct_light_ray = Ray(hit.value().hit_p, scattered_mat.value().wo);
    std::optional<HitInfo> hit_next_bounce
        = bvh.hit<std::optional<HitInfo>>(direct_light_ray, thread_stack, prims);

    // if next ray hits an object
    if (hit_next_bounce.has_value()) {
      // ray hits a light
      if (hit_next_bounce.value().mat->is_emissive()) {
        if (mat_sample_pdf != 0) {
          light_pdf
              = hit_next_bounce.value().obj->pdf(hit.value().hit_p, hit_next_bounce.value().hit_p,
                                                 scattered_mat.value().wo)
                / lights.num_lights();

          float mis_weight = mat_sample_pdf / (light_pdf + mat_sample_pdf);

          bounce_result
              += throughput * mis_weight
                 * hit_next_bounce.value().mat->emitted(direct_light_ray, hit_next_bounce.value());
        } else {
          bounce_result
              += throughput
                 * hit_next_bounce.value().mat->emitted(direct_light_ray, hit_next_bounce.value());
        }
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
  }
  return bounce_result;
}