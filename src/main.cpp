#include <chrono>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <bvh.h>
#include <fmt/chrono.h>
#include <fmt/core.h>
#include <geometry/surface.h>
#include <integrators.h>
#include <json_scene.h>
#include <material/material.h>
#include <stb_image_write.h>
#include <tl_camera.h>
#include <tonemapper.h>

#include <algorithm>
#include <glm/glm.hpp>

void setup_for_bvh(const std::vector<std::unique_ptr<Surface>>& list_objects,
                   std::vector<AABB>& bboxes, std::vector<glm::vec3>& centers) {
  for (auto const& obj : list_objects) {
    // get AABB
    bboxes.push_back(obj->bounds());
    // get center
    centers.push_back(obj->get_center());
  }
}

int main(int argc, char* argv[]) {
  // if CHANNEL_NUM is 4, you can use alpha channel in png
  constexpr uint8_t CHANNEL_NUM = 3;
  constexpr uint32_t NUM_BINS = 16;
  std::string json_file_path;
  integrator_data rendering_settings;

  std::vector<std::unique_ptr<Material>> mat_list;
  std::vector<std::unique_ptr<Surface>> list_objects;

  std::vector<AABB> list_bboxes;
  std::vector<glm::vec3> list_centers;

  if (argc <= 1) {
    fmt::println("Scene file path argument was not given");
    return 0;
  }

  json_file_path = argv[1];

  /*
    parse the file, load objects and materials
    list_objects and mat_list will be constant after this. ptr_to_objects especially depends on
    this
  */
  bool scene_load_check
      = set_scene_from_json(json_file_path, rendering_settings, list_objects, mat_list);

  if (!scene_load_check) {
    fmt::println("Scene was not loaded");
    return 0;
  }

  // take a list of Surfaces and make a vector of AABBs and centers
  setup_for_bvh(list_objects, list_bboxes, list_centers);

  // make bvh
  const BVH bvh = BVH::build(list_bboxes, list_centers, NUM_BINS);

  auto begin_time = std::chrono::steady_clock::now();
  std::vector<glm::vec3> acc_image;

  // run integrator
  switch (rendering_settings.func) {
    case integrator_func::normal:
      acc_image = scene_integrator(rendering_settings, bvh, list_objects, normal_integrator);
      break;

    case integrator_func::material:
      acc_image = scene_integrator(rendering_settings, bvh, list_objects, material_integrator);
      break;
  }

  auto end_time = std::chrono::steady_clock::now();
  auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - begin_time);

  auto duration_secs = std::chrono::duration_cast<std::chrono::seconds>(duration_ms);
  duration_ms -= std::chrono::duration_cast<std::chrono::milliseconds>(duration_secs);
  auto duration_mins = std::chrono::duration_cast<std::chrono::minutes>(duration_secs);
  duration_secs -= std::chrono::duration_cast<std::chrono::seconds>(duration_mins);

  fmt::print("Render time: {} {} {}\n", duration_mins, duration_secs, duration_ms);

  // TODO apply tone mapper
  simple_gamma_correction(acc_image);

  // buffer for image writing
  uint8_t* pixels = new uint8_t[rendering_settings.resolution.x * rendering_settings.resolution.y
                                * CHANNEL_NUM];

  // clamp pixel values to [0,255] before writing to png
  int index = 0;
  for (size_t i = 0; i < acc_image.size(); i++) {
    pixels[index++] = std::clamp(static_cast<int>(255.99 * acc_image[i][0]), 0, 255);
    pixels[index++] = std::clamp(static_cast<int>(255.99 * acc_image[i][1]), 0, 255);
    pixels[index++] = std::clamp(static_cast<int>(255.99 * acc_image[i][2]), 0, 255);
  }

  stbi_write_png("v_img.png", rendering_settings.resolution.x, rendering_settings.resolution.y,
                 CHANNEL_NUM, pixels, rendering_settings.resolution.x * CHANNEL_NUM);

  delete[] pixels;

  return 0;
}
