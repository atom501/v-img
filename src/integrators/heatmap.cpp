#include <fmt/core.h>
#include <integrators.h>

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

std::vector<glm::vec3> heatmap_img(const integrator_data& render_data, const BVH& bvh,
                                   const std::vector<std::unique_ptr<Surface>>& prims,
                                   const int max_count) {
  constexpr int sqrt_pixel_samples = 4;
  constexpr float inv_sqrt_sample = 1 / static_cast<float>(sqrt_pixel_samples);

  const uint32_t image_width = render_data.resolution[0];
  const uint32_t image_height = render_data.resolution[1];

  std::atomic_uint pixels_done = 0;
  const unsigned int total_pixels = image_width * image_height;

  const unsigned int num_cores = std::thread::hardware_concurrency();

  std::vector<uint32_t> num_hits_accumulated(total_pixels);
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
      uint32_t pixel_hit_accumulator;

      // make a stack for each core. instead of one for each bvh hit call
      std::vector<size_t> thread_stack;
      thread_stack.reserve(64);

      for (size_t y_start = 0; y_start < image_height; y_start += 8) {
        for (size_t x_start = 8 * c; x_start < image_width; x_start += 8 * num_cores) {
          const auto& bottom_x = x_start;
          const auto& top_x = std::min(x_start + 7, static_cast<size_t>(image_width - 1));

          const auto& bottom_y = y_start;
          const auto& top_y = std::min(y_start + 7, static_cast<size_t>(image_height - 1));

          for (size_t y = bottom_y; y <= top_y; y++) {
            for (size_t x = bottom_x; x <= top_x; x++) {
              pixel_hit_accumulator = 0;
              // init hash at the start of each work
              size_t image_index = x + ((image_height - 1 - y) * image_width);
              pcg32_srandom_r(&pcg_state, image_index, c);

              // stratified samples. Fixed to 4x4 for heatmap
              for (size_t x_sample = 0; x_sample < sqrt_pixel_samples; x_sample++) {
                for (size_t y_sample = 0; y_sample < sqrt_pixel_samples; y_sample++) {
                  float rand_x = inv_sqrt_sample * (x_sample + rand_float(pcg_state));
                  float rand_y = inv_sqrt_sample * (y_sample + rand_float(pcg_state));

                  // make ray with random offset
                  Ray cam_ray = render_data.camera.generate_ray(x + rand_x, y + rand_y);

                  // use set integrator to get color for a pixel
                  pixel_hit_accumulator += bvh.hit_heatmap(cam_ray, thread_stack, prims);
                }
              }

              // average final sum
              num_hits_accumulated[image_index]
                  = static_cast<uint32_t>(pixel_hit_accumulator / 16.f);

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

  uint32_t find_max = 0;
  // find max if set max_count to 0
  if (max_count == 0) {
    for (const auto& c : num_hits_accumulated) {
      if (c > find_max) {
        find_max = c;
      }
    }
  } else {
    find_max = max_count;
  }

  fmt::println("Max primitive heat by a ray: {}", find_max);

  std::vector<glm::vec3> image(total_pixels);

  for (size_t i = 0; i < total_pixels; i++) {
    float pixel_heat = static_cast<float>(num_hits_accumulated[i]) / static_cast<float>(find_max);

    image[i] = hue_to_rgb((1.f - pixel_heat) * 0.8333f);
  }

  return image;
}
