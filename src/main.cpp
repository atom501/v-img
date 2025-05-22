#include <chrono>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <bvh.h>
#include <fmt/chrono.h>
#include <fmt/core.h>
#include <geometry/group_emitters.h>
#include <geometry/mesh.h>
#include <geometry/surface.h>
#include <integrators.h>
#include <material/material.h>
#include <scene_loading/json_scene.h>
#include <scene_loading/mitsuba_scene.h>
#include <stb_image_write.h>
#include <texture.h>
#include <tl_camera.h>
#include <tonemapper.h>
#include <glm/gtx/transform.hpp>

#include <algorithm>
#include <args.hxx>
#include <filesystem>
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
  integrator_data rendering_settings;

  int heatmap_max = -1;

  std::vector<std::unique_ptr<Material>> mat_list;
  std::vector<std::unique_ptr<Surface>> list_objects;
  std::vector<Surface*> list_lights;
  std::vector<std::unique_ptr<Mesh>> list_meshes;
  std::vector<std::unique_ptr<Texture>> texture_list;

  std::vector<AABB> list_bboxes;
  std::vector<glm::vec3> list_centers;

  // parsing CLI arguments
  args::ArgumentParser parser("CPU raytracer", "");
  args::HelpFlag help(parser, "help", "Display this help menu", {'h', "help"});
  args::ValueFlag<std::string> filename(parser, "file", "Scene filename", {'f'});
  args::ValueFlag<int> heatmap(
      parser, "heatmap",
      "Enable heatmap mode. Number of primitives to set as the max. 0 uses maximum from scene",
      {'m'});

  try {
    parser.ParseCLI(argc, argv);
  } catch (args::Help) {
    std::cout << parser;
    return 0;
  } catch (args::ParseError e) {
    std::cerr << e.what() << std::endl;
    std::cerr << parser;
    return 1;
  } catch (args::ValidationError e) {
    std::cerr << e.what() << std::endl;
    std::cerr << parser;
    return 1;
  }

  if (!filename) {
    fmt::println("Scene file path argument (-f) was not given");
    return 0;
  }

  if (heatmap) {
    heatmap_max = args::get(heatmap);
  }

  std::filesystem::path scene_file_path = args::get(filename);

  /*
    parse the file, load objects and materials
    list_objects and mat_list will be constant after this. ptr_to_objects especially depends on
    this
  */
  std::filesystem::path extension = scene_file_path.extension();
  bool scene_load_check;

  if (extension.string() == ".json") {
    scene_load_check = set_scene_from_json(scene_file_path, rendering_settings, list_objects,
                                           mat_list, list_lights, list_meshes, texture_list);
  } else if (extension.string() == ".xml") {
    scene_load_check = set_scene_from_xml(scene_file_path, rendering_settings, list_objects,
                                          mat_list, list_lights, list_meshes, texture_list);
  }

  if (!scene_load_check) {
    fmt::println("Scene was not loaded");
    return 0;
  }

  GroupOfEmitters lights = GroupOfEmitters(list_lights);

  fmt::println("Number of lights {}", lights.num_lights());

  // take a list of Surfaces and make a vector of AABBs and centers
  setup_for_bvh(list_objects, list_bboxes, list_centers);

  // make bvh
  BVH bvh = BVH::build(list_bboxes, list_centers, NUM_BINS);
  // bvh.stack.reserve(64);

  fmt::println("scene BVH built");

  auto begin_time = std::chrono::steady_clock::now();
  std::vector<glm::vec3> acc_image;

  fmt::println("Start render");

  if (heatmap_max < 0) {
    // run integrator
    switch (rendering_settings.func) {
      case integrator_func::normal:
        acc_image
            = scene_integrator(rendering_settings, bvh, list_objects, lights, normal_integrator);
        break;

      case integrator_func::material:
        acc_image
            = scene_integrator(rendering_settings, bvh, list_objects, lights, material_integrator);
        break;
      case integrator_func::mis:
        acc_image = scene_integrator(rendering_settings, bvh, list_objects, lights, mis_integrator);
        break;
    }
  } else {
    fmt::println("Heatmap mode");
    acc_image = heatmap_img(rendering_settings, bvh, list_objects, heatmap_max);
  }

  auto end_time = std::chrono::steady_clock::now();
  auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - begin_time);

  auto duration_secs = std::chrono::duration_cast<std::chrono::seconds>(duration_ms);
  duration_ms -= std::chrono::duration_cast<std::chrono::milliseconds>(duration_secs);
  auto duration_mins = std::chrono::duration_cast<std::chrono::minutes>(duration_secs);
  duration_secs -= std::chrono::duration_cast<std::chrono::seconds>(duration_mins);

  fmt::print("Render time: {} {} {}\n", duration_mins, duration_secs, duration_ms);

  // TODO apply tone mapper
  sRGB_gamma_correction(acc_image);

  // buffer for image writing
  uint8_t* pixels = new uint8_t[rendering_settings.resolution.x * rendering_settings.resolution.y
                                * CHANNEL_NUM];

  // clamp pixel values to [0,255] before writing to png
  int index = 0;
  for (size_t i = 0; i < acc_image.size(); i++) {
    // if NaN, write magenta
    if (std::isnan(acc_image[i][0]) || std::isnan(acc_image[i][1]) || std::isnan(acc_image[i][2])) {
      pixels[index++] = 255;
      pixels[index++] = 0;
      pixels[index++] = 255;
    } else {
      // multiply by 255.999 so rounding occurs correctly
      pixels[index++] = std::clamp(static_cast<int>(255.999 * acc_image[i][0]), 0, 255);
      pixels[index++] = std::clamp(static_cast<int>(255.999 * acc_image[i][1]), 0, 255);
      pixels[index++] = std::clamp(static_cast<int>(255.999 * acc_image[i][2]), 0, 255);
    }
  }

  stbi_write_png("v_img.png", rendering_settings.resolution.x, rendering_settings.resolution.y,
                 CHANNEL_NUM, pixels, rendering_settings.resolution.x * CHANNEL_NUM);

  delete[] pixels;

  return 0;
}
