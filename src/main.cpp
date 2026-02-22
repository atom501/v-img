#include <chrono>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <bvh.h>
#include <color_utils.h>
#include <fmt/core.h>
#include <geometry/emitters.h>
#include <geometry/mesh.h>
#include <geometry/surface.h>
#include <integrators.h>
#include <material/material.h>
#include <scene_loading/gltf_loading.h>
#include <scene_loading/json_scene.h>
#include <scene_loading/mitsuba_scene.h>
#include <stb_image_write.h>
#include <texture.h>
#include <tl_camera.h>

#include <algorithm>
#include <args.hxx>
#include <filesystem>
#include <glm/glm.hpp>
#include <glm/gtx/transform.hpp>

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
  int16_t num_threads = -1;
  tonemapper tonemapping_func = tonemapper::clamp;

  std::vector<std::unique_ptr<Material>> mat_list;
  std::vector<std::unique_ptr<Surface>> list_objects;
  std::vector<Emitter*> list_lights;
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
  args::ValueFlag<int16_t> numthreads(parser, "threads", "Number of threads to be used", {'t'});
  args::ValueFlag<int16_t> set_tonemapper(
      parser, "tonemapper",
      "Set tonemapper to be used. 0 for clamp, 1 for agx, 2 for reinhard, 3 for aces", {'c'});
  args::ValueFlag<std::string> set_jsonfilename(
      parser, "j_file", "Json file with extra settings for gltf scenes", {'j'});
  args::ValueFlag<std::string> set_debug_params(
      parser, "debug", "Only trace one pixel. Input value is 'x y'. out image x, height - y",
      {'d'});
  args::ValueFlag<int> bvh_type(parser, "bvh", "Type of BVH to build. 0 for binned, 1 for sweepSAH",
                                {'b'});

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

  if (numthreads) {
    num_threads = args::get(numthreads);
  }

  if (set_tonemapper) {
    int16_t val = args::get(set_tonemapper);

    if (val >= 0 && val < static_cast<int16_t>(tonemapper::COUNT)) {
      tonemapping_func = static_cast<tonemapper>(val);
    }
  }

  std::filesystem::path scene_file_path = args::get(filename);

  /*
    parse the file, load objects and materials
    list_objects and mat_list will be constant after this. ptr_to_objects especially depends on
    this
  */
  std::filesystem::path extension = scene_file_path.extension();
  bool scene_load_check = false;
  auto begin_time = std::chrono::steady_clock::now();

  if (extension.string() == ".json") {
    scene_load_check = set_scene_from_json(scene_file_path, rendering_settings, list_objects,
                                           mat_list, list_lights, list_meshes, texture_list);
  } else if (extension.string() == ".xml") {
    scene_load_check = set_scene_from_mitsuba_xml(scene_file_path, rendering_settings, list_objects,
                                                  mat_list, list_lights, list_meshes, texture_list);
  } else if (extension.string() == ".gltf" || extension.string() == ".glb") {
    // load parameters not set for gltf
    std::filesystem::path json_gltf_file = set_jsonfilename ? args::get(set_jsonfilename) : "";

    std::optional<std::string> json_string_opt;
    if (json_gltf_file.string() != "") {
      json_string_opt = read_file(json_gltf_file);
    }

    nlohmann::json json_settings;
    if (json_string_opt.has_value()) {
      json_settings = nlohmann::json::parse(json_string_opt.value());
    }

    scene_load_check
        = set_scene_from_gltf(scene_file_path, rendering_settings, list_objects, mat_list,
                              list_lights, list_meshes, texture_list, json_settings);
  }

  auto end_time = std::chrono::steady_clock::now();

  print_time_taken(begin_time, end_time, "scene loading");

  if (!scene_load_check) {
    fmt::println("Scene was not loaded");
    return 0;
  }

#ifdef __AVX2__
  fmt::println("Raytracer will use AVX2");
#else
  fmt::println("AVX2 not supported or is disabled");
#endif

  GroupOfEmitters lights = GroupOfEmitters(list_lights);

  rendering_settings.num_threads = num_threads;

  fmt::println("\nNumber of lights loaded {}", lights.num_lights());
  fmt::println("Number of Textures loaded {}", texture_list.size());
  fmt::println("Number of Meshes loaded {}", list_meshes.size());
  fmt::println("Number of Surfaces loaded {}", list_objects.size());

  fmt::println("\nImage resolution {}x{}, samples per pixel {}, max ray depth {}\n",
               rendering_settings.resolution.x, rendering_settings.resolution.y,
               rendering_settings.samples, rendering_settings.depth);

  // take a list of Surfaces and make a vector of AABBs and centers
  setup_for_bvh(list_objects, list_bboxes, list_centers);

  int bvh_to_build = 0;
  if (bvh_type) {
    bvh_to_build = args::get(bvh_type);
  }

  // make bvh
  begin_time = std::chrono::steady_clock::now();
  BVH bvh;

  if (bvh_to_build == 0) {
    fmt::println("Building binned BVH");
    bvh = BVH::build_bin_bvh(list_bboxes, list_centers, NUM_BINS);
  } else if (bvh_to_build == 1) {
    fmt::println("Building sweep BVH");
    std::vector<size_t> temp_indices(list_bboxes.size());
    std::iota(temp_indices.begin(), temp_indices.end(), 0);

    bvh = BVH::build_sweep_bvh(list_bboxes, list_centers, temp_indices, 8);
  } else {
    fmt::println("Building binned BVH");
    bvh = BVH::build_bin_bvh(list_bboxes, list_centers, NUM_BINS);
  }

  end_time = std::chrono::steady_clock::now();

  print_time_taken(begin_time, end_time, "BVH construction");
  fmt::println("BVH max depth {}\n", bvh.max_depth);

  begin_time = std::chrono::steady_clock::now();
  std::vector<glm::vec3> acc_image;

  fmt::println("Started rendering\n");

  if (!set_debug_params) {
    if (heatmap_max < 0) {
      // run integrator
      switch (rendering_settings.func) {
        case integrator_func::s_normal:
          fmt::println("Running shading normal integrator");
          acc_image = scene_integrator(rendering_settings, bvh, list_objects, lights,
                                       shading_normal_integrator);
          break;
        case integrator_func::g_normal:
          fmt::println("Running geometric normal integrator");
          acc_image = scene_integrator(rendering_settings, bvh, list_objects, lights,
                                       geometric_normal_integrator);
          break;
        case integrator_func::material:
          fmt::println("Running material integrator");
          acc_image = scene_integrator(rendering_settings, bvh, list_objects, lights,
                                       material_integrator);
          break;
        case integrator_func::mis:
          fmt::println("Running MIS integrator");
          acc_image
              = scene_integrator(rendering_settings, bvh, list_objects, lights, mis_integrator);
          break;
      }
    } else {
      fmt::println("Creating Heatmap for ray intersection");
      acc_image = heatmap_img(rendering_settings, bvh, list_objects, heatmap_max);
    }
  } else {
    glm::vec3 pixel_out;
    std::string input_str = args::get(set_debug_params);

    size_t first_space_idx = input_str.find_first_of(' ');

    if (first_space_idx == std::string::npos) {
      fmt::println("value of y not found");
      return 0;
    }

    int x = std::stoi(input_str.substr(0, first_space_idx));
    int y = std::stoi(input_str.substr(first_space_idx + 1));

    switch (rendering_settings.func) {
      case integrator_func::s_normal:
        fmt::println("Running shading normal integrator");
        pixel_out = trace_pixel(rendering_settings, bvh, list_objects, lights,
                                shading_normal_integrator, x, y);
        break;
      case integrator_func::g_normal:
        fmt::println("Running geometric normal integrator");
        pixel_out = trace_pixel(rendering_settings, bvh, list_objects, lights,
                                geometric_normal_integrator, x, y);
        break;
      case integrator_func::material:
        fmt::println("Running material integrator");
        pixel_out
            = trace_pixel(rendering_settings, bvh, list_objects, lights, material_integrator, x, y);
        break;
      case integrator_func::mis:
        fmt::println("Running MIS integrator");
        pixel_out
            = trace_pixel(rendering_settings, bvh, list_objects, lights, mis_integrator, x, y);
        break;
    }

    acc_image.push_back(pixel_out);

    fmt::println("Value of pixel x {} y {} with {} samples in linear space is ({}, {}, {})", x, y,
                 rendering_settings.samples, pixel_out.x, pixel_out.y, pixel_out.z);
  }

  end_time = std::chrono::steady_clock::now();

  print_time_taken(begin_time, end_time, "image rendering");

  // apply tonemapper
  switch (tonemapping_func) {
    case tonemapper::clamp:
      fmt::println("Render image clamped");
      simple_clamp(acc_image);
      break;
    case tonemapper::agx:
      fmt::println("agx tonemapper applied");
      agx(acc_image);
      break;
    case tonemapper::reinhard:
      fmt::println("reinhard tonemapper applied");
      reinhard_lum(acc_image);
      break;
    case tonemapper::aces:
      fmt::println("aces tonemapper applied");
      aces(acc_image);
      break;

    default:
      fmt::println("No tonemapper applied");
      break;
  }

  // apply gamma correction
  sRGB_gamma_correction(acc_image);

  if (set_debug_params) {
    fmt::println("Value of pixel in rgb space is ({}, {}, {})", acc_image[0].x, acc_image[0].y,
                 acc_image[0].z);

    return 0;
  }

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

  auto now_utc = std::chrono::utc_clock::now();
  std::string out_file_timestamp = std::format(
      "{:%Y-%m-%d_%H-%M-%S}", std::chrono::time_point_cast<std::chrono::seconds>(now_utc));

  std::string img_filename = std::format("v_img_{}.png", out_file_timestamp);

  int write_img = stbi_write_png(img_filename.c_str(), rendering_settings.resolution.x,
                                 rendering_settings.resolution.y, CHANNEL_NUM, pixels,
                                 rendering_settings.resolution.x * CHANNEL_NUM);

  if (write_img) {
    fmt::println("output image written to {}", img_filename);
  } else {
    fmt::println("failed to write output image");
  }

  delete[] pixels;

  return 0;
}
