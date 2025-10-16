#include <integrators.h>

#include <glm/gtx/norm.hpp>

static inline float balance_heuristic(float pdf1, float pdf2) { return pdf1 / (pdf1 + pdf2); }

static inline float geometric_term(const glm::vec3& look_from, const glm::vec3& point_on_surface,
                                   const glm::vec3& surface_normal) {
  glm::vec3 dir_from_surf = look_from - point_on_surface;
  float distance2 = glm::length2(dir_from_surf);
  dir_from_surf = glm::normalize(dir_from_surf);

  float cosine = std::abs(glm::dot(surface_normal, dir_from_surf));

  return cosine / distance2;
}

glm::vec3 mis_integrator(Ray& input_ray, std::vector<size_t>& thread_stack, const BVH& bvh,
                         const std::vector<std::unique_ptr<Surface>>& prims,
                         const GroupOfEmitters& lights, pcg32_random_t& hash_state, uint32_t depth,
                         Background* background) {
  Ray test_ray = input_ray;
  float light_pdf, mat_pdf;
  glm::vec3 bounce_result = glm::vec3(0.0f);
  glm::vec3 throughput = glm::vec3(1.0f);
  size_t d = 0;

  // tracking eta_scale and removing it from the path contribution when doing russian roulette
  float eta_scale = 1;
  constexpr uint32_t roulette_threshold = 5;

  // perform initial hit test
  std::optional<HitInfo> hit = bvh.hit<std::optional<HitInfo>>(test_ray, thread_stack, prims);

  if (!hit.has_value()) {
    // scene missed. if background not assigned it will be init to black
    return background->background_emit(test_ray);
  } else if (hit.value().mat->is_emissive()) {
    // if first hit is emissive
    return hit.value().mat->emitted(test_ray, hit.value().hit_n_s, hit.value().hit_p);
  }

  // If first hit does not miss or hit light
  for (d = 0; d < depth; d++) {
    bool mat_is_delta = hit.value().mat->is_delta();

    // skip if light sampling material with a delta functions where material pdf = 0
    if (!mat_is_delta) {
      // light sampling
      auto [light_col, l_sample_info] = lights.sample(hit.value().hit_p, hash_state);

      // pdf == 0 where light sampling fails
      if (l_sample_info.pdf != 0.f) {
        Ray shadow_ray = Ray(hit.value().hit_p, l_sample_info.wi);
        shadow_ray.maxT = l_sample_info.dist - 0.0001f;

        // check if ray hits anything between ray origin and the point on light
        bool light_is_occluded = bvh.occlude(shadow_ray, thread_stack, prims);

        // if light visible from point
        if (!light_is_occluded) {
          const auto [mat_eval, mat_pdf] = hit.value().mat->eval_pdf_pair(
              test_ray.dir, l_sample_info.wi, hit.value(),
              propagate_reflect_cone(test_ray.ray_cone, hit.value().angle_spread,
                                     l_sample_info.dist));

          if (mat_pdf != 0 && !std::isnan(mat_pdf)) {
            float G = l_sample_info.G;
            float mis_weight = balance_heuristic(l_sample_info.pdf, mat_pdf * G);
            bounce_result += throughput * mat_eval * mis_weight * G * light_col / l_sample_info.pdf;
          }
        }
      }
    }

    // info for next bounce
    std::optional<ScatterInfo> scattered_mat
        = hit.value().mat->sample_mat(test_ray.dir, hit.value(), hash_state);

    if (!scattered_mat.has_value()) {
      return bounce_result;
    }

    if (scattered_mat.value().eta != 0.f) {
      eta_scale /= (scattered_mat.value().eta * scattered_mat.value().eta);
      test_ray.ray_cone = propagate_refract_cone(
          test_ray.ray_cone, test_ray.dir, hit.value().hit_p, hit.value().angle_spread,
          scattered_mat.value().eta, scattered_mat.value().wo);
    } else {
      float hit_dist = glm::length(test_ray.o - hit.value().hit_p);
      test_ray.ray_cone
          = propagate_reflect_cone(test_ray.ray_cone, hit.value().angle_spread, hit_dist);
    }

    float mat_sample_pdf
        = hit.value().mat->pdf(test_ray.dir, scattered_mat.value().wo, hit.value());

    // material sampling
    throughput *= hit.value().mat->eval_div_pdf(test_ray.dir, scattered_mat.value().wo, hit.value(),
                                                test_ray.ray_cone);

    // where light goes after hitting object
    Ray direct_light_ray = Ray(hit.value().hit_p, scattered_mat.value().wo, test_ray.ray_cone);
    std::optional<HitInfo> hit_next_bounce
        = bvh.hit<std::optional<HitInfo>>(direct_light_ray, thread_stack, prims);

    // if next ray hits an object
    if (hit_next_bounce.has_value()) {
      // ray hits a light
      if (hit_next_bounce.value().mat->is_emissive()) {
        if (mat_sample_pdf != 0) {
          light_pdf
              = hit_next_bounce.value().obj->surf_pdf(
                    hit.value().hit_p, hit_next_bounce.value().hit_p, scattered_mat.value().wo)
                / lights.num_lights();

          float G = geometric_term(hit.value().hit_p, hit_next_bounce.value().hit_p,
                                   hit_next_bounce.value().hit_n_g);

          float mis_weight = balance_heuristic(mat_sample_pdf * G, light_pdf);

          bounce_result += throughput * mis_weight
                           * hit_next_bounce.value().mat->emitted(direct_light_ray,
                                                                  hit_next_bounce.value().hit_n_s,
                                                                  hit_next_bounce.value().hit_p);
        } else {
          // don't MIS sample for delta function
          bounce_result += throughput
                           * hit_next_bounce.value().mat->emitted(direct_light_ray,
                                                                  hit_next_bounce.value().hit_n_s,
                                                                  hit_next_bounce.value().hit_p);
        }
        // stop bounce since emit encountered
        return bounce_result;
      } else {
        // perform russian roulette
        if (d > roulette_threshold) {
          // random [0,1) float
          float rand_float = static_cast<float>(pcg32_random_r(&hash_state))
                             / std::numeric_limits<uint32_t>::max();

          glm::vec3 rr_throughput = (1.f / eta_scale) * throughput;
          float max_val = std::min(
              std::max(std::max(rr_throughput.x, rr_throughput.y), rr_throughput.z), 0.95f);

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
      // missed scene. compensate for hitting emissive background
      if (mat_sample_pdf != 0 && background->is_emissive()) {
        light_pdf = background->background_pdf(direct_light_ray.dir) / lights.num_lights();

        // G = 1
        float mis_weight = balance_heuristic(mat_sample_pdf, light_pdf);

        bounce_result += throughput * mis_weight * background->background_emit(direct_light_ray);
      }
      return bounce_result;
    }
  }
  return bounce_result;
}