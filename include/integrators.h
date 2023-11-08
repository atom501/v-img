#pragma once

#include <geometry/sphere.h>
#include <rng/lcg_rand.h>
#include <tl_camera.h>

#include <algorithm>
#include <cstdint>
#include <functional>
#include <glm/glm.hpp>
#include <iostream>
#include <thread>
#include <vector>

struct integrator_data {
  glm::ivec2 resolution;
  uint32_t samples;
  uint32_t depth;
  TLCam camera;
};

template <typename F> std::vector<glm::vec3> scene_integrator(const integrator_data& render_data,
                                                              const Sphere& s, F integrator) {
  const unsigned int image_width = render_data.resolution[0];
  const unsigned int image_height = render_data.resolution[1];

  const int num_cores = std::thread::hardware_concurrency();
  unsigned int run_threads = 0;

  // image width * height
  std::vector<glm::vec3> image_accumulated(image_width * image_height);
  std::vector<std::thread> workers;

  for (int h = image_height - 1; h >= 0; h -= num_cores) {
    run_threads = std::min(num_cores, h + 1);

    for (int j = 0; j < run_threads; j++) {
      unsigned int assigned_h = h - j;

      workers.push_back(std::thread([&, assigned_h]() {
        for (unsigned int w = 0; w < image_width; ++w) {
          glm::vec3 pixel_col_accumulator = glm::vec3(0.0f);
          // init hash at the start of each thread
          int32_t init_hash = lcg::hash(w, assigned_h, 1);

          for (int sample = 0; sample < render_data.samples; sample++) {
            float rand_x = lcg::unitrand(init_hash);
            float rand_y = lcg::unitrand(init_hash);

            // make ray with random offset
            Ray cam_ray = render_data.camera.generate_ray(w + rand_x, assigned_h + rand_y);

            // use set integrator to get color for a pixel
            pixel_col_accumulator += integrator(cam_ray, s, init_hash, render_data.depth);
          }
          // average final sum
          pixel_col_accumulator /= render_data.samples;

          image_accumulated[w + ((image_height - 1 - assigned_h) * image_width)]
              = pixel_col_accumulator;
        }
      }));
    }

    for (std::thread& thread : workers) {
      thread.join();
    }

    workers.clear();
  }

  return image_accumulated;
}

glm::vec3 normal_integrator(Ray& input_ray, const Sphere& s, int32_t& hash_value, uint32_t depth);

glm::vec3 material_integrator(Ray& input_ray, const Sphere& s, int32_t& hash_value, uint32_t depth);
