#pragma once

#include <bvh.h>
#include <geometry/group_emitters.h>
#include <geometry/surface.h>
#include <progress_print.h>
#include <rng/pcg_rand.h>
#include <tl_camera.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <glm/glm.hpp>
#include <thread>
#include <utility>
#include <vector>

enum class integrator_func { normal, material, mis };

struct integrator_data {
  integrator_func func;
  glm::ivec2 resolution;
  uint32_t samples;
  uint32_t depth;
  glm::vec3 background_col;
  TLCam camera;
};

template <typename F>
std::vector<glm::vec3> scene_integrator(const integrator_data& render_data, const BVH& bvh,
                                        const std::vector<std::unique_ptr<Surface>>& prims,
                                        const GroupOfEmitters& lights, F integrator) {
  const uint32_t image_width = render_data.resolution[0];
  const uint32_t image_height = render_data.resolution[1];

  std::atomic_uint pixels_done = 0;
  const unsigned int total_pixels = image_width * image_height;

  const unsigned int num_cores = std::thread::hardware_concurrency();

  std::vector<glm::vec3> image_accumulated(total_pixels);
  std::vector<std::thread> workers;
  std::vector<std::pair<glm::u16vec2, glm::u16vec2>> work_list;

  // make the array used to divide work. Each entry contains 8x8 region of the image

  for (size_t x = 0; x < image_width; x += 8) {
    for (size_t y = 0; y < image_height; y += 8) {
      auto bottom_pixel = glm::u16vec2(x, y);
      auto top_pixel = glm::u16vec2(std::min(x + 7, static_cast<size_t>(image_width - 1)),
                                    std::min(y + 7, static_cast<size_t>(image_height - 1)));

      work_list.push_back(std::make_pair(bottom_pixel, top_pixel));
    }
  }

  const unsigned int work_list_len = work_list.size();

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
    fmt::println("done");
    fflush(stdout);
    fmt::print("\n");
  });

  for (unsigned int c = 0; c < num_cores; c++) {
    workers.push_back(std::thread([&, c, work_list_len]() {
      pcg32_random_t pcg_state;
      glm::vec3 pixel_col_accumulator;

      for (size_t i = c; i < work_list_len; i += num_cores) {
        std::pair<glm::u16vec2, glm::u16vec2>& curr_work = work_list[i];
        const auto& bottom_x = curr_work.first[0];
        const auto& top_x = curr_work.second[0];

        const auto& bottom_y = curr_work.first[1];
        const auto& top_y = curr_work.second[1];

        for (size_t y = bottom_y; y <= top_y; y++) {
          for (size_t x = bottom_x; x <= top_x; x++) {
            pixel_col_accumulator = glm::vec3(0.0f);
            // init hash at the start of each work
            size_t image_index = x + ((image_height - 1 - y) * image_width);
            pcg32_srandom_r(&pcg_state, image_index, c);

            for (int sample = 0; sample < render_data.samples; sample++) {
              float rand_x = static_cast<float>(pcg32_random_r(&pcg_state))
                             / std::numeric_limits<uint32_t>::max();
              float rand_y = static_cast<float>(pcg32_random_r(&pcg_state))
                             / std::numeric_limits<uint32_t>::max();

              // make ray with random offset
              Ray cam_ray = render_data.camera.generate_ray(x + rand_x, y + rand_y);

              // use set integrator to get color for a pixel
              pixel_col_accumulator
                  += integrator(cam_ray, bvh, prims, lights, pcg_state, render_data.depth);
            }
            // average final sum
            pixel_col_accumulator /= render_data.samples;

            image_accumulated[image_index] = pixel_col_accumulator;
            ++pixels_done;
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

  return image_accumulated;
}

glm::vec3 normal_integrator(Ray& input_ray, const BVH& bvh,
                            const std::vector<std::unique_ptr<Surface>>& prims,
                            const GroupOfEmitters& lights, pcg32_random_t& hash_state,
                            uint32_t depth);

glm::vec3 material_integrator(Ray& input_ray, const BVH& bvh,
                              const std::vector<std::unique_ptr<Surface>>& prims,
                              const GroupOfEmitters& lights, pcg32_random_t& hash_state,
                              uint32_t depth);

glm::vec3 mis_integrator(Ray& input_ray, const BVH& bvh,
                         const std::vector<std::unique_ptr<Surface>>& prims,
                         const GroupOfEmitters& lights, pcg32_random_t& hash_state, uint32_t depth);
