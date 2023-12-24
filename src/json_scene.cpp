#include <fmt/core.h>
#include <json_scene.h>

#include <filesystem>
#include <fstream>
#include <string>

static std::string read_file(std::filesystem::path path) {
  // Open the stream to 'lock' the file.
  std::ifstream f(path, std::ios::in | std::ios::binary);

  // Obtain the size of the file.
  const auto file_size = std::filesystem::file_size(path);

  // Create a buffer.
  std::string json_string(file_size, '\0');

  // Read the whole file into the buffer.
  f.read(json_string.data(), file_size);

  return json_string;
}

static glm::mat4 look_from_json(const nlohmann::json& json_cam_xform) {
  glm::vec3 look_from = glm::vec3(0);
  glm::vec3 look_at = glm::vec3(0);
  glm::vec3 look_up = glm::vec3(0, 1, 0);

  if (json_cam_xform.contains("from")) {
    look_from[0] = json_cam_xform["from"][0];
    look_from[1] = json_cam_xform["from"][1];
    look_from[2] = json_cam_xform["from"][2];

    fmt::println("look from ({}, {}, {}) set", look_from[0], look_from[1], look_from[2]);
  } else {
    fmt::println("No camera from point given. Default (0,0,0) set");
  }

  if (json_cam_xform.contains("at")) {
    look_at[0] = json_cam_xform["at"][0];
    look_at[1] = json_cam_xform["at"][1];
    look_at[2] = json_cam_xform["at"][2];

    fmt::println("look at ({}, {}, {}) set", look_at[0], look_at[1], look_at[2]);
  } else {
    fmt::println("No camera at point given. Default (0,0,0) set");
  }

  if (json_cam_xform.contains("up")) {
    look_up[0] = json_cam_xform["up"][0];
    look_up[1] = json_cam_xform["up"][1];
    look_up[2] = json_cam_xform["up"][2];

    fmt::println("look up ({}, {}, {}) set", look_up[0], look_up[1], look_up[2]);
  } else {
    fmt::println("No camera at vector given. Default (0,1,0) set");
  }

  return camToWorld(look_from, look_at, look_up);
}

bool set_integrator_data(const nlohmann::json& json_settings, integrator_data& integrator_data) {
  if (json_settings.contains("camera")) {
    glm::mat4 cam_xform;
    const nlohmann::json cam_set = json_settings["camera"];
    glm::ivec2 res = glm::ivec2(500, 500);

    if (cam_set.contains("resolution")) {
      res[0] = cam_set["resolution"][0];
      res[1] = cam_set["resolution"][1];
      fmt::println("resolution {}, {} set", res[0], res[1]);
    } else {
      fmt::println("Resolution not given. Default 500, 500 set");
    }
    integrator_data.resolution = res;

    if (cam_set.contains("transform")) {
      cam_xform = look_from_json(cam_set["transform"]);
    } else {
      fmt::println("Camera transform not given");
      return false;
    }

    const float vfov = cam_set.value("vfov", 40.0f);
    integrator_data.camera = TLCam(cam_xform, res, vfov);
  } else {
    fmt::println("Camera settings not given");
    return false;
  }

  uint32_t samples = 30;
  uint32_t depth = 30;

  if (json_settings.contains("sampler")) {
    auto sampler_json = json_settings["sampler"];

    samples = sampler_json.value("samples", samples);
    depth = sampler_json.value("depth", depth);
  } else {
    fmt::println("Sampler not given. Default 30 samples and 30 depth set");
  }

  integrator_data.samples = samples;
  integrator_data.depth = depth;

  glm::vec3 background_color = glm::vec3(0);
  if (json_settings.contains("background")) {
    background_color[0] = json_settings["background"][0];
    background_color[1] = json_settings["background"][1];
    background_color[2] = json_settings["background"][2];
  }
  integrator_data.background_col = background_color;

  integrator_func func = integrator_func::normal;
  if (json_settings.contains("integrator")) {
    std::string integrator_string = json_settings["integrator"]["type"];
    fmt::println("function type read {}", integrator_string);

    if (integrator_string == "normal") {
      func = integrator_func::normal;
      fmt::println("Normal integrator set");
    } else if (integrator_string == "material") {
      func = integrator_func::material;
      fmt::println("Material integrator set");
    }
  } else {
    fmt::println("Integrator function not given. Default normal integrator set");
  }

  integrator_data.func = func;

  return true;
}

bool set_scene_from_json(const std::string& path_file, integrator_data& integrator_data) {
  // parse json at path_file
  const auto json_string = read_file(path_file);
  const nlohmann::json json_settings = nlohmann::json::parse(json_string);

  // set integrator data
  const bool data_set_check = set_integrator_data(json_settings, integrator_data);

  if (!data_set_check) {
    return false;
  }

  fmt::println("Integrator data set");

  return true;
}