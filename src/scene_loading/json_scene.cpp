#include <background.h>
#include <fmt/core.h>
#include <geometry/mesh.h>
#include <geometry/sphere.h>
#include <geometry/triangle.h>
#include <material/dielectric.h>
#include <material/diffuse_light.h>
#include <material/lambertian.h>
#include <material/principled.h>
#include <scene_loading/json_scene.h>
#include <texture.h>

#include <filesystem>
#include <fstream>
#include <glm/gtx/transform.hpp>

namespace glm {

  void from_json(const nlohmann::json& j, vec2& vec) {
    j[0].get_to(vec.x);
    j[1].get_to(vec.y);
  }

  void from_json(const nlohmann::json& j, vec3& vec) {
    j[0].get_to(vec.x);
    j[1].get_to(vec.y);
    j[2].get_to(vec.z);
  }

  void from_json(const nlohmann::json& j, quat& q) {
    j[0].get_to(q.x);
    j[1].get_to(q.y);
    j[2].get_to(q.z);
    j[3].get_to(q.w);
  }

}  // namespace glm

std::optional<std::string> read_file(std::filesystem::path path) {
  if (!std::filesystem::exists(path)) {
    fmt::println("Json scene file does not exists");
    return std::nullopt;
  }

  try {
    // Open the stream to 'lock' the file.
    std::ifstream file(path, std::ios::in | std::ios::binary);

    // Obtain the size of the file.
    const auto file_size = std::filesystem::file_size(path);

    // Create a buffer.
    std::string json_string(file_size, '\0');

    // Read the whole file into the buffer.
    file.read(json_string.data(), file_size);

    file.close();

    return json_string;
  } catch (const std::exception& e) {
    fmt::println("Exception reading file {}", e.what());
    return std::nullopt;
  }
}

static glm::mat4 get_transform(const nlohmann::json& json_xform) {
  glm::mat4 xform = glm::mat4(1.0f);

  if (!json_xform.contains("transform")) {
    return xform;
  }

  const auto& json_array = json_xform["transform"];

  // loop over and apply transforms to xform
  for (auto& transform_entry : json_array) {
    if (transform_entry.contains("scale")) {
      if (transform_entry["scale"].is_array()) {
        glm::vec3 scale = transform_entry["scale"].template get<glm::vec3>();
        xform = glm::scale(scale) * xform;
      } else {
        float scale = transform_entry["scale"];
        xform = glm::scale(glm::vec3(scale)) * xform;
      }

    } else if (transform_entry.contains("rotate")) {
      glm::quat rotate_quat = transform_entry["rotate"].template get<glm::quat>();
      xform = glm::toMat4(rotate_quat) * xform;

    } else if (transform_entry.contains("translate")) {
      glm::vec3 translate = transform_entry["translate"].template get<glm::vec3>();
      xform = glm::translate(translate) * xform;

    } else if (transform_entry.contains("x") | transform_entry.contains("y")
               | transform_entry.contains("z") | transform_entry.contains("o")) {
      glm::vec3 x(1.f, 0.f, 0.f);
      glm::vec3 y(0.f, 1.f, 0.f);
      glm::vec3 z(0.f, 0.f, 1.f);
      glm::vec3 o(0.f, 0.f, 0.f);

      if (transform_entry.contains("x")) {
        x = transform_entry["x"].template get<glm::vec3>();
      } else if (transform_entry.contains("y")) {
        y = transform_entry["y"].template get<glm::vec3>();
      } else if (transform_entry.contains("z")) {
        z = transform_entry["z"].template get<glm::vec3>();
      } else if (transform_entry.contains("z")) {
        o = transform_entry["o"].template get<glm::vec3>();
      }

      glm::mat4 loaded_mat
          = glm::mat4(glm::vec4(x, 0.f), glm::vec4(y, 0.f), glm::vec4(z, 0.f), glm::vec4(o, 1.f));
      xform = loaded_mat * xform;
    } else {
      fmt::println("Unknown transformation type when reading json. {}", transform_entry.dump());
    }
  }

  return xform;
}

static glm::mat4 look_from_json(const nlohmann::json& json_cam_xform) {
  glm::vec3 look_from = glm::vec3(0);
  glm::vec3 look_at = glm::vec3(0);
  glm::vec3 look_up = glm::vec3(0, 1, 0);

  if (json_cam_xform.contains("from")) {
    look_from = json_cam_xform["from"].template get<glm::vec3>();

    fmt::println("look from ({}, {}, {}) set", look_from[0], look_from[1], look_from[2]);
  } else {
    fmt::println("No camera from point given. Default (0,0,0) set");
  }

  if (json_cam_xform.contains("at")) {
    look_at = json_cam_xform["at"].template get<glm::vec3>();

    fmt::println("look at ({}, {}, {}) set", look_at[0], look_at[1], look_at[2]);
  } else {
    fmt::println("No camera at point given. Default (0,0,0) set");
  }

  if (json_cam_xform.contains("up")) {
    look_up = json_cam_xform["up"].template get<glm::vec3>();

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
      res = cam_set["resolution"].template get<glm::vec2>();
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
    integrator_data.camera = TLCam(cam_xform, res, vfov, 0.f, 1.f);
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
  } else {
    fmt::println("Sampler not given. Default 30 samples and 30 depth set");
  }

  integrator_data.samples = samples;
  integrator_data.depth = depth;

  // set background color
  glm::vec3 background_color = glm::vec3(0);
  if (json_settings.contains("background")) {
    background_color = json_settings["background"].template get<glm::vec3>();
  }
  integrator_data.background = std::make_unique<ConstBackground>(ConstBackground(glm::vec3(0)));

  // set scene integrator
  integrator_func func = integrator_func::s_normal;
  if (json_settings.contains("integrator")) {
    std::string integrator_string = json_settings["integrator"]["type"];

    if (integrator_string == "s_normal") {
      func = integrator_func::s_normal;
    } else if (integrator_string == "g_normal") {
      func = integrator_func::g_normal;
    } else if (integrator_string == "material") {
      func = integrator_func::material;
    } else if (integrator_string == "mis") {
      func = integrator_func::mis;
    } else {
      fmt::println("Integrator {} is not defined. Set to shading normal", integrator_string);
    }
  } else {
    fmt::println("Integrator function not given. Default shading normal integrator set");
  }

  integrator_data.func = func;

  return true;
}

/*
 * Takes in json as input. Check if object uses a texure already in the list. If not add new one
 * to the list and return pointer
 */
Texture* json_to_texture(const nlohmann::json& mat_json,
                         std::vector<std::unique_ptr<Texture>>& texture_list) {
  // if can't find "texture" object assume constant texture
  if (!mat_json.contains("texture")) {
    glm::vec3 albedo = mat_json["albedo"].template get<glm::vec3>();
    ConstColor col(albedo);

    texture_list.push_back(std::make_unique<ConstColor>(col));

    return texture_list[texture_list.size() - 1].get();
  } else {
    nlohmann::json tex_data = mat_json["texture"];

    if (tex_data["type"] == "constant") {
      glm::vec3 albedo = tex_data["albedo"].template get<glm::vec3>();
      ConstColor col(albedo);

      texture_list.push_back(std::make_unique<ConstColor>(col));

      return texture_list[texture_list.size() - 1].get();
    } else if (tex_data["type"] == "checkered") {
      uint32_t width = tex_data["width"];
      uint32_t height = tex_data["height"];

      Checkerboard cb(width, height, tex_data["col1"].template get<glm::vec3>(),
                      tex_data["col2"].template get<glm::vec3>());

      texture_list.push_back(std::make_unique<Checkerboard>(cb));

      return texture_list[texture_list.size() - 1].get();
    }  // TODO else if check texture name is present. to reuse a texture

    return nullptr;
  }
}

bool set_list_of_materials(const nlohmann::json& json_settings,
                           std::vector<std::unique_ptr<Material>>& list_materials,
                           std::unordered_map<std::string, size_t>& name_to_index,
                           std::vector<std::unique_ptr<Texture>>& texture_list) {
  if (json_settings.contains("materials")) {
    auto json_mat_list = json_settings["materials"];

    for (auto& mat_data : json_mat_list) {
      if (mat_data["type"] == "lambertian") {
        Texture* t = json_to_texture(mat_data, texture_list);

        list_materials.push_back(std::make_unique<Lambertian>(t));
        name_to_index[mat_data["name"]] = list_materials.size() - 1;

      } else if (mat_data["type"] == "diffuse_light") {
        list_materials.push_back(std::make_unique<DiffuseLight>(mat_data));
        name_to_index[mat_data["name"]] = list_materials.size() - 1;

      } else if (mat_data["type"] == "dielectric") {
        list_materials.push_back(std::make_unique<Dielectric>(mat_data));
        name_to_index[mat_data["name"]] = list_materials.size() - 1;

      } else if (mat_data["type"] == "principled") {
        glm::vec3 base_col = mat_data["base_color"].template get<glm::vec3>();

        float roughness = mat_data.value("roughness", 0.5f);
        float anisotropic = mat_data.value("anisotropic", 0.f);
        float eta = mat_data.value("eta", 1.5f);
        float subsurface = mat_data.value("subsurface", 0.f);
        float metallic = mat_data.value("metallic", 0.f);

        float spec_trans = mat_data.value("spec_trans", 0.f);
        float specular = mat_data.value("specular", 0.5f);
        float spec_tint = mat_data.value("spec_tint", 0.f);

        float sheen = mat_data.value("sheen", 0.f);
        float sheen_tint = mat_data.value("sheen_tint", 0.5f);

        float clearcoat = mat_data.value("clearcoat", 0.f);
        float clearcoat_gloss = mat_data.value("clearcoat_gloss", 1.f);

        texture_list.push_back(std::make_unique<ConstColor>(base_col));

        list_materials.push_back(std::make_unique<Principled>(
            texture_list.back().get(), spec_trans, metallic, subsurface, specular, roughness,
            spec_tint, anisotropic, sheen, sheen_tint, clearcoat, clearcoat_gloss, eta));
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
                         std::vector<Emitter*>& list_lights,
                         const std::unordered_map<std::string, size_t>& name_to_index,
                         std::vector<std::unique_ptr<Mesh>>& list_meshes) {
  if (!json_settings.contains("surfaces")) {
    fmt::println("Json file does not contain surfaces");
    return false;
  }
  auto json_surface_list = json_settings["surfaces"];

  for (auto& surf_data : json_surface_list) {
    glm::mat4 surf_xform = get_transform(surf_data);
    Material* mat_ptr = list_materials[name_to_index.at(surf_data["mat_name"])].get();

    if (surf_data["type"] == "quad") {
      // create quad
      Mesh quad_mesh = create_quad_mesh(mat_ptr, surf_xform);
      list_meshes.push_back(std::make_unique<Mesh>(quad_mesh));

      add_tri_list_to_scene(quad_mesh, list_surfaces, list_meshes.back().get(), list_lights);
    } else if (surf_data["type"] == "sphere") {
      glm::vec3 center = surf_data["center"].template get<glm::vec3>();
      float radius = surf_data.value("radius", 1.0f);

      auto sphere = Sphere(center, radius, mat_ptr);
      list_surfaces.push_back(std::make_unique<Sphere>(sphere));

      if (mat_ptr->is_emissive()) {
        Sphere* s_ptr = static_cast<Sphere*>(list_surfaces[list_surfaces.size() - 1].get());
        list_lights.push_back(s_ptr);
      }
    } else if (surf_data["type"] == "mesh") {
      std::filesystem::path model_filename = surf_data["filename"];
      std::filesystem::path scene_filename = json_settings["scene_file_path"];

      const auto model_path_rel_file = scene_filename.remove_filename() / model_filename;

      // data for mesh object
      std::vector<glm::vec3> vertices;
      std::vector<glm::vec3> normals;
      std::vector<glm::vec2> texcoords;

      std::vector<std::array<uint32_t, 3>> indices;

      load_from_obj(model_path_rel_file.string(), indices, vertices, normals, texcoords,
                    surf_xform);

      auto tri_mesh = Mesh(indices, vertices, normals, texcoords, mat_ptr);
      list_meshes.push_back(std::make_unique<Mesh>(tri_mesh));

      add_tri_list_to_scene(tri_mesh, list_surfaces, list_meshes.back().get(), list_lights);
    } else {
      fmt::println("Unknown surface {}", surf_data["type"]);
      return false;
    }
  }

  return true;
}

bool set_scene_from_json(const std::filesystem::path& path_file, integrator_data& integrator_data,
                         std::vector<std::unique_ptr<Surface>>& list_surfaces,
                         std::vector<std::unique_ptr<Material>>& list_materials,
                         std::vector<Emitter*>& list_lights,
                         std::vector<std::unique_ptr<Mesh>>& list_meshes,
                         std::vector<std::unique_ptr<Texture>>& texture_list) {
  // parse json at path_file
  const auto json_string_opt = read_file(path_file);
  std::string json_string;

  if (!json_string_opt.has_value()) {
    return false;
  } else {
    json_string = json_string_opt.value();
  }

  nlohmann::json json_settings = nlohmann::json::parse(json_string);

  json_settings["scene_file_path"] = path_file;

  // set integrator data
  if (!set_integrator_data(json_settings, integrator_data)) {
    fmt::println("Integrator data set failed");
    return false;
  }

  // set material list and create hashmap for mapping material name to index in list
  std::unordered_map<std::string, size_t> name_to_mat;
  if (set_list_of_materials(json_settings, list_materials, name_to_mat, texture_list)) {
    fmt::println("List of materials loaded");
  } else {
    fmt::println("Material loading failed");
    return false;
  }

  // initialise objects and add to list. If it has emissive material add it's pointer to light
  // list
  if (set_list_of_objects(json_settings, list_surfaces, list_materials, list_lights, name_to_mat,
                          list_meshes)) {
    fmt::println("List of surfaces loaded");
  } else {
    fmt::println("Surface loading failed");
    return false;
  }

  return true;
}