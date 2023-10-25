#include <hit_utils.h>
#include <integrators.h>
#include <rng/lcg_rand.h>

#include <cstdint>
#include <optional>

// returns vector of size image width * height * channels. Where each value [0,1]
std::vector<glm::vec3> normal_integrator(const integrator_data& render_data, const TLCam& camera,
                                         const Sphere& s) {
  const int image_width = render_data.resolution[0];
  const int image_height = render_data.resolution[1];

  // image width * height * channels
  std::vector<glm::vec3> image_accumulated(image_width * image_height);

  // TODO use multi sample

  Ray cam_ray;
  std::optional<HitInfo> hit;

  for (int j = image_height - 1; j >= 0; --j) {
    for (int i = 0; i < image_width; ++i) {
      int32_t init_hash = lcg::hash(i, j, 1);

      for (int sample = 0; sample < render_data.samples; sample++) {
        float rand_x = lcg::unitrand(init_hash);
        float rand_y = lcg::unitrand(init_hash);

        Ray cam_ray = camera.generate_ray(i + rand_x, j + rand_y);

        hit = s.hit(cam_ray);

        // If hit then get color using normal
        if (hit.has_value()) {
          auto normal_col = hit.value().hit_n;
          normal_col = (normal_col + 1.0f) / 2.0f;

          image_accumulated[i + ((image_height - 1 - j) * image_width)] += normal_col;
        } else {
          // Else set gradient
          glm::vec3 unit_dir = glm::normalize(cam_ray.dir);
          float a = 0.5 * (unit_dir[1] + 1.0);
          glm::vec3 col = (1.0f - a) * glm::vec3(1.0, 1.0, 1.0) + a * glm::vec3(0.5, 0.7, 1.0);

          image_accumulated[i + ((image_height - 1 - j) * image_width)] += col;
        }
      }

      image_accumulated[i + ((image_height - 1 - j) * image_width)] /= render_data.samples;
    }
  }

  return image_accumulated;
}