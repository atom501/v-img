#include <fmt/core.h>
#include <geometry/mesh.h>
#include <geometry/quads.h>
#include <geometry/sphere.h>
#include <geometry/triangle.h>
#include <json_scene.h>
#include <material/dielectric.h>
#include <material/diffuse_light.h>
#include <material/lambertian.h>
#include <texture.h>
#define TINYOBJLOADER_IMPLEMENTATION
#include <tiny_obj_loader.h>

#include <filesystem>
#include <fstream>
#include <glm/gtx/transform.hpp>

namespace glm {

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

static std::optional<std::string> read_file(std::filesystem::path path) {
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
    fmt::println("Transform entry {}", transform_entry.dump());

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

/*
 * Takes in json as input. Check if object uses a texure already in the list. If not add new one to
 * the list and return pointer
 */
Texture* json_to_texture(const nlohmann::json& mat_json,
                         std::vector<std::unique_ptr<Texture>>& texture_list) {
  glm::vec3 albedo = mat_json["albedo"].template get<glm::vec3>();
  ConstColor col(albedo);

  texture_list.push_back(std::make_unique<ConstColor>(col));

  return texture_list[texture_list.size() - 1].get();
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
        auto temp_lambertian = Lambertian(t);

        // add material to list and associate a name with the index
        list_materials.push_back(std::make_unique<Lambertian>(temp_lambertian));
        name_to_index[mat_data["name"]] = list_materials.size() - 1;
      } else if (mat_data["type"] == "diffuse_light") {
        auto temp_light = DiffuseLight(mat_data);

        // add material to list and associate a name with the index
        list_materials.push_back(std::make_unique<DiffuseLight>(temp_light));
        name_to_index[mat_data["name"]] = list_materials.size() - 1;
      } else if (mat_data["type"] == "dielectric") {
        auto temp_dielectric = Dielectric(mat_data);

        // add material to list and associate a name with the index
        list_materials.push_back(std::make_unique<Dielectric>(temp_dielectric));
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
                         const std::unordered_map<std::string, size_t>& name_to_index,
                         std::vector<std::unique_ptr<Mesh>>& list_meshes) {
  if (json_settings.contains("surfaces")) {
    auto json_surface_list = json_settings["surfaces"];

    for (auto& surf_data : json_surface_list) {
      if (surf_data["type"] == "quad") {
        glm::mat4 surf_xform = get_transform(surf_data);

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

        // transform the quad
        glm::vec4 temp_o = surf_xform * glm::vec4(l_corner, 1.0f);
        temp_o /= temp_o[3];
        l_corner = temp_o;

        u = glm::vec3(surf_xform * glm::vec4(u, 0.0f));
        v = glm::vec3(surf_xform * glm::vec4(v, 0.0f));

        std::string mat_name = surf_data["mat_name"];
        Material* mat_ptr = list_materials[name_to_index.at(mat_name)].get();

        auto quad = Quad(l_corner, u, v, mat_ptr);
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

        auto sphere = Sphere(center, radius, mat_ptr);
        list_surfaces.push_back(std::make_unique<Sphere>(sphere));

        if (mat_ptr->is_emissive()) {
          Surface* s_ptr = list_surfaces[list_surfaces.size() - 1].get();
          list_lights.push_back(s_ptr);
        }
      } else if (surf_data["type"] == "mesh") {
        tinyobj::attrib_t attrib;
        std::vector<tinyobj::shape_t> shapes;
        std::vector<tinyobj::material_t> materials;
        std::string warnings;
        std::string errors;

        std::filesystem::path model_filename = surf_data["filename"];
        std::filesystem::path scene_filename = json_settings["scene_file_path"];

        const auto model_path_rel_file = scene_filename.remove_filename() / model_filename;

        if (tinyobj::LoadObj(&attrib, &shapes, &materials, &warnings, &errors,
                             model_path_rel_file.string().c_str())
            == false) {
          fmt::println("Tinyobj failed to load the mesh \n {}", errors);
          return false;
        }

        // data for mesh object
        std::vector<glm::vec3> vertices;
        std::vector<glm::vec3> normals;
        std::vector<uint32_t> tri_vertex;
        std::vector<uint32_t> tri_normal;

        glm::mat4 surf_xform = get_transform(surf_data);
        // read vertices and normals, also transform them
        for (size_t i = 0; i < attrib.vertices.size(); i += 3) {
          auto vec = glm::vec3(attrib.vertices[i], attrib.vertices[i + 1], attrib.vertices[i + 2]);
          glm::vec4 result = surf_xform * glm::vec4(vec, 1);
          result /= result.w;
          vertices.push_back(glm::vec3(result));
        }

        const glm::mat4 normal_xform = glm::transpose(glm::inverse(surf_xform));
        for (size_t i = 0; i < attrib.normals.size(); i += 3) {
          auto norm = glm::vec3(attrib.normals[i], attrib.normals[i + 1], attrib.normals[i + 2]);
          glm::vec4 result = normal_xform * glm::vec4(norm, 0);
          normals.push_back(glm::normalize(glm::vec3(result)));
        }

        vertices.shrink_to_fit();
        normals.shrink_to_fit();

        for (const auto& shape : shapes) {
          const std::vector<tinyobj::index_t>& indices = shape.mesh.indices;
          const std::vector<int>& material_ids = shape.mesh.material_ids;

          // for each triangle get the index in vector of vertices for a vertex
          for (size_t i = 0; i < material_ids.size(); i++) {
            // for ith triangle

            // get index for each vertex
            tri_vertex.push_back(indices[3 * i].vertex_index);
            tri_vertex.push_back(indices[3 * i + 1].vertex_index);
            tri_vertex.push_back(indices[3 * i + 2].vertex_index);

            auto tri_min_point = glm::min(
                vertices[tri_vertex[3 * i]],
                glm::min(vertices[tri_vertex[3 * i + 1]], vertices[tri_vertex[(3 * i) + 2]]));

            auto tri_max_point = glm::max(
                vertices[tri_vertex[3 * i]],
                glm::max(vertices[tri_vertex[3 * i + 1]], vertices[tri_vertex[(3 * i) + 2]]));

            // get index for each vertex normal

            // if one of the vertex normals don't exist use face normal
            if (indices[3 * i].normal_index == -1 || indices[3 * i + 1].normal_index == -1
                || indices[3 * i + 2].normal_index == -1) {
              glm::vec3 p0 = vertices[tri_vertex[3 * i]], p1 = vertices[tri_vertex[3 * i + 1]],
                        p2 = vertices[tri_vertex[3 * i + 2]];
              auto edge1 = p1 - p0;
              auto edge2 = p2 - p0;

              glm::vec3 face_normal = glm::normalize(glm::cross(edge1, edge2));

              normals.push_back(face_normal);

              // calculate face normal and assign it to all normal index for triangle
              tri_normal.push_back(normals.size() - 1);
              tri_normal.push_back(normals.size() - 1);
              tri_normal.push_back(normals.size() - 1);
            } else {
              tri_normal.push_back(indices[3 * i].normal_index);
              tri_normal.push_back(indices[3 * i + 1].normal_index);
              tri_normal.push_back(indices[3 * i + 2].normal_index);
            }
          }
        }

        std::string mat_name = surf_data["mat_name"];
        Material* mat_ptr = list_materials[name_to_index.at(mat_name)].get();

        auto tri_mesh = Mesh(vertices, tri_vertex, normals, tri_normal, mat_ptr);
        list_meshes.push_back(std::make_unique<Mesh>(tri_mesh));

        // add to list of surfaces
        for (size_t i = 0; i < tri_vertex.size() / 3; i++) {
          auto tri = Triangle(list_meshes[list_meshes.size() - 1].get(), i, mat_ptr);
          list_surfaces.push_back(std::make_unique<Triangle>(tri));
        }

        // add to list of lights if needed
        size_t rev_count_index = list_surfaces.size() - 1;

        if (mat_ptr->is_emissive()) {
          for (size_t i = 0; i < tri_vertex.size(); i++, rev_count_index--) {
            Surface* s_ptr = list_surfaces[rev_count_index].get();
            list_lights.push_back(s_ptr);
          }
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

bool set_scene_from_json(const std::filesystem::path& path_file, integrator_data& integrator_data,
                         std::vector<std::unique_ptr<Surface>>& list_surfaces,
                         std::vector<std::unique_ptr<Material>>& list_materials,
                         std::vector<Surface*>& list_lights,
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
  if (set_integrator_data(json_settings, integrator_data)) {
    fmt::println("Integrator data is set");
  } else {
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