#include <fmt/core.h>
#include <integrators.h>

#include <algorithm>

static float min3(float a, float b, float c) { return std::min(std::min(a, b), c); }

static glm::vec3 hue_to_rgb(float h) {
  auto kr = fmod(5 + h * 6, 6);
  auto kg = fmod(3 + h * 6, 6);
  auto kb = fmod(1 + h * 6, 6);

  auto r = 1 - std::max(min3(kr, 4 - kr, 1), 0.f);
  auto g = 1 - std::max(min3(kg, 4 - kg, 1), 0.f);
  auto b = 1 - std::max(min3(kb, 4 - kb, 1), 0.f);

  return glm::vec3(r, g, b);
}

glm::vec3 turbo_colormap(float x) {
  // Source: https://research.google/blog/turbo-an-improved-rainbow-colormap-for-visualization/

  const glm::vec4 kRedVec4 = glm::vec4(0.13572138, 4.61539260, -42.66032258, 132.13108234);
  const glm::vec4 kGreenVec4 = glm::vec4(0.09140261, 2.19418839, 4.84296658, -14.18503333);
  const glm::vec4 kBlueVec4 = glm::vec4(0.10667330, 12.64194608, -60.58204836, 110.36276771);
  const glm::vec2 kRedVec2 = glm::vec2(-152.94239396, 59.28637943);
  const glm::vec2 kGreenVec2 = glm::vec2(4.27729857, 2.82956604);
  const glm::vec2 kBlueVec2 = glm::vec2(-89.90310912, 27.34824973);

  x = std::clamp(x, 0.f, 1.f);
  glm::vec4 v4 = glm::vec4(1.0, x, x * x, x * x * x);
  glm::vec2 v2 = glm::vec2(v4.z, v4.w) * v4.z;

  return glm::vec3(dot(v4, kRedVec4) + dot(v2, kRedVec2), dot(v4, kGreenVec4) + dot(v2, kGreenVec2),
                   dot(v4, kBlueVec4) + dot(v2, kBlueVec2));
}

std::vector<glm::vec3> heatmap_img(const integrator_data& render_data, const BVH& bvh,
                                   const std::vector<std::unique_ptr<Surface>>& prims,
                                   float factor) {
  const uint32_t image_width = render_data.resolution[0];
  const uint32_t image_height = render_data.resolution[1];

  std::atomic_uint pixels_done = 0;
  const unsigned int total_pixels = image_width * image_height;

  const unsigned int num_cores = std::thread::hardware_concurrency();

  std::vector<float> num_hits_accumulated(total_pixels);
  std::vector<std::thread> workers;
  std::vector<std::pair<glm::u16vec2, glm::u16vec2>> work_list;

  timer_killer progress_bar;

  // thread to print progress
  std::thread print_progress([&, total_pixels]() {
    unsigned int pixels_done_copy = 0;
    float progress = 0;

    fmt::print("\r0 % done");
    fflush(stdout);

    while (progress_bar.wait_for(std::chrono::milliseconds(800))) {
      pixels_done_copy = pixels_done.load();
      progress = (static_cast<float>(pixels_done_copy) / total_pixels) * 100.0f;

      fmt::print("\r{:.2f} % done", progress);
      fflush(stdout);
    }
    fmt::print("\n");
    fmt::println("Render Completed");
    fflush(stdout);
    fmt::print("\n");
  });

  for (unsigned int c = 0; c < num_cores; c++) {
    workers.push_back(std::thread([&, c]() {
      pcg32_random_t pcg_state;
      float pixel_hit_accumulator;

      // make a stack for each core. instead of one for each bvh hit call
      std::vector<size_t> thread_stack;
      thread_stack.reserve(64);

      for (size_t y_start = 0; y_start < image_height; y_start += 8) {
        for (size_t x_start = 8 * c; x_start < image_width; x_start += 8 * num_cores) {
          const auto& bottom_x = x_start;
          const auto top_x = std::min(x_start + 7, static_cast<size_t>(image_width - 1));

          const auto& bottom_y = y_start;
          const auto top_y = std::min(y_start + 7, static_cast<size_t>(image_height - 1));

          for (size_t y = bottom_y; y <= top_y; y++) {
            for (size_t x = bottom_x; x <= top_x; x++) {
              pixel_hit_accumulator = 0.f;
              // init hash at the start of each work
              size_t image_index = x + ((image_height - 1 - y) * image_width);
              size_t image_seq_start = x + y;

              pcg32_srandom_r(&pcg_state, image_index, 0);

              // spp fixed for heatmap
              for (size_t s = 0; s < render_data.samples; s++) {
                glm::vec2 pixel_offset = random_x_y_r2(image_seq_start + s);

                // make ray with random offset
                Ray cam_ray
                    = render_data.camera.generate_ray(x + pixel_offset.x, y + pixel_offset.y,
                                                      rand_float(pcg_state), rand_float(pcg_state));

                // use set integrator to get color for a pixel
                pixel_hit_accumulator += bvh.hit<float>(cam_ray, thread_stack, prims);
              }

              // average final sum
              num_hits_accumulated[image_index]
                  = static_cast<uint32_t>(pixel_hit_accumulator / render_data.samples);

              ++pixels_done;
            }
          }
        }
      }
    }));
  }

  for (std::thread& thread : workers) {
    thread.join();
  }

  progress_bar.kill();
  print_progress.join();

  std::vector<glm::vec3> image(total_pixels);

  if (factor <= 0) {
    factor = 20.f;
  }

  fmt::println("Heatmap factor set to {}", factor);

  for (size_t i = 0; i < total_pixels; i++) {
    image[i] = turbo_colormap(num_hits_accumulated[i] / factor);
  }

  return image;
}
