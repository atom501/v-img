#include <fmt/core.h>
#include <geometry/quads.h>
#include <geometry/sphere.h>
#include <json_scene.h>
#include <material/diffuse_light.h>
#include <material/lambertian.h>

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

    // set resolution
    if (cam_set.contains("resolution")) {
      res[0] = cam_set["resolution"][0];
      res[1] = cam_set["resolution"][1];
      fmt::println("resolution {}, {} set", res[0], res[1]);
    } else {
      fmt::println("Resolution not given. Default 500, 500 set");
    }
    integrator_data.resolution = res;

    // set camera transform
    if (cam_set.contains("transform")) {
      cam_xform = look_from_json(cam_set["transform"]);
    } else {
      fmt::println("Camera transform not given");
      return false;
    }

    // set vertical fov
    const float vfov = cam_set.value("vfov", 40.0f);
    integrator_data.camera = TLCam(cam_xform, res, vfov);
  } else {
    fmt::println("Camera settings not given");
    return false;
  }

  uint32_t samples = 30;
  uint32_t depth = 30;

  // set sample per pixel and max ray depth
  if (json_settings.contains("sampler")) {
    auto sampler_json = json_settings["sampler"];

    samples = sampler_json.value("samples", samples);
    depth = sampler_json.value("depth", depth);

    fmt::println("Samples {} and max depth {}", samples, depth);
  } else {
    fmt::println("Sampler not given. Default 30 samples and 30 depth set");
  }

  integrator_data.samples = samples;
  integrator_data.depth = depth;

  // set background color
  glm::vec3 background_color = glm::vec3(0);
  if (json_settings.contains("background")) {
    background_color[0] = json_settings["background"][0];
    background_color[1] = json_settings["background"][1];
    background_color[2] = json_settings["background"][2];
  }
  integrator_data.background_col = background_color;

  // set scene integrator
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
    } else if (integrator_string == "mis") {
      func = integrator_func::mis;
      fmt::println("MIS integrator set");
    }
  } else {
    fmt::println("Integrator function not given. Default normal integrator set");
  }

  integrator_data.func = func;

  return true;
}

bool set_list_of_materials(const nlohmann::json& json_settings,
                           std::vector<std::unique_ptr<Material>>& list_materials,
                           std::unordered_map<std::string, size_t>& name_to_index) {
  if (json_settings.contains("materials")) {
    auto json_mat_list = json_settings["materials"];

    for (auto& mat_data : json_mat_list) {
      if (mat_data["type"] == "lambertian") {
        auto temp_lambertian = Lambertian(mat_data);

        // add material to list and associate a name with the index
        list_materials.push_back(std::make_unique<Lambertian>(temp_lambertian));
        name_to_index[mat_data["name"]] = list_materials.size() - 1;
      } else if (mat_data["type"] == "diffuse_light") {
        auto temp_light = DiffuseLight(mat_data);

        // add material to list and associate a name with the index
        list_materials.push_back(std::make_unique<DiffuseLight>(temp_light));
        name_to_index[mat_data["name"]] = list_materials.size() - 1;
      } else {
        std::string surf_name = mat_data["type"];
        fmt::println("Unknown material {}", surf_name);
        return false;
      }
    }
    return true;
  } else
    return false;
}

bool set_list_of_objects(const nlohmann::json& json_settings,
                         std::vector<std::unique_ptr<Surface>>& list_surfaces,
                         std::vector<std::unique_ptr<Material>>& list_materials,
                         std::vector<Surface*>& list_lights,
                         const std::unordered_map<std::string, size_t>& name_to_index) {
  if (json_settings.contains("surfaces")) {
    auto json_surface_list = json_settings["surfaces"];

    for (auto& surf_data : json_surface_list) {
      if (surf_data["type"] == "quad") {
        // TODO add transformation init from json
        glm::mat4 surf_transform = glm::mat4(1.0f);
        glm::vec3 l_corner;
        l_corner[0] = surf_data["l_corner"][0];
        l_corner[1] = surf_data["l_corner"][1];
        l_corner[2] = surf_data["l_corner"][2];

        glm::vec3 u;
        u[0] = surf_data["u"][0];
        u[1] = surf_data["u"][1];
        u[2] = surf_data["u"][2];

        glm::vec3 v;
        v[0] = surf_data["v"][0];
        v[1] = surf_data["v"][1];
        v[2] = surf_data["v"][2];

        std::string mat_name = surf_data["mat_name"];

        Material* mat_ptr = list_materials[name_to_index.at(mat_name)].get();
        auto quad = Quad(l_corner, u, v, mat_ptr, surf_transform);
        list_surfaces.push_back(std::make_unique<Quad>(quad));

        if (mat_ptr->is_emissive()) {
          Surface* s_ptr = list_surfaces[list_surfaces.size() - 1].get();
          list_lights.push_back(s_ptr);
        }
      } else if (surf_data["type"] == "sphere") {
        glm::mat4 surf_transform = glm::mat4(1.0f);

        glm::vec3 center;

        center[0] = surf_data["center"][0];
        center[1] = surf_data["center"][1];
        center[2] = surf_data["center"][2];

        float radius = surf_data.value("radius", 1.0f);
        std::string mat_name = surf_data["mat_name"];

        Material* mat_ptr = list_materials[name_to_index.at(mat_name)].get();
        auto sphere = Sphere(center, radius, mat_ptr, surf_transform);
        list_surfaces.push_back(std::make_unique<Sphere>(sphere));

        if (mat_ptr->is_emissive()) {
          Surface* s_ptr = list_surfaces[list_surfaces.size() - 1].get();
          list_lights.push_back(s_ptr);
        }
      } else {
        std::string surf_name = surf_data["type"];
        fmt::println("Unknown surface {}", surf_name);
        return false;
      }
    }
    return true;
  } else
    return false;
}

bool set_scene_from_json(const std::string& path_file, integrator_data& integrator_data,
                         std::vector<std::unique_ptr<Surface>>& list_surfaces,
                         std::vector<std::unique_ptr<Material>>& list_materials,
                         std::vector<Surface*>& list_lights) {
  // parse json at path_file
  const auto json_string = read_file(path_file);
  const nlohmann::json json_settings = nlohmann::json::parse(json_string);

  // set integrator data
  if (set_integrator_data(json_settings, integrator_data)) {
    fmt::println("Integrator data is set");
  } else {
    fmt::println("Integrator data set failed");
    return false;
  }

  // set material list and create hashmap for mapping material name to index in list
  std::unordered_map<std::string, size_t> name_to_mat;
  if (set_list_of_materials(json_settings, list_materials, name_to_mat)) {
    fmt::println("List of materials loaded");
  } else {
    fmt::println("Material loading failed");
    return false;
  }

  // initialise objects and add to list. If it has emissive material add it's pointer to light list
  if (set_list_of_objects(json_settings, list_surfaces, list_materials, list_lights, name_to_mat)) {
    fmt::println("List of surfaces loaded");
  } else {
    fmt::println("Surface loading failed");
    return false;
  }

  return true;
}