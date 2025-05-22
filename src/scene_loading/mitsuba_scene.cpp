#include <geometry/quads.h>
#include <material/diffuse_light.h>
#include <material/lambertian.h>
#include <scene_loading/mitsuba_scene.h>

Material* mat_index_from_obj(std::shared_ptr<tinyparser_mitsuba::Object> mat_obj,
                             std::vector<std::unique_ptr<Texture>>& texture_list,
                             std::vector<std::unique_ptr<Material>>& list_materials,
                             std::unordered_map<std::string, size_t>& name_to_mat) {
  std::string plugin_type = mat_obj->pluginType();
  std::string bsdf_name = mat_obj->id();

  if (auto findit = name_to_mat.find(bsdf_name); findit != name_to_mat.end()) {
    // if mat already found just assign
    size_t mat_id = name_to_mat[findit->first];
    return list_materials[mat_id].get();
  } else {
    std::string plugin_type = mat_obj->pluginType();

    if (plugin_type != "twosided") {
      // make material
      if (mat_obj->type() == tinyparser_mitsuba::OT_BSDF) {
        if (plugin_type == "diffuse") {
          auto properties = mat_obj->properties();
          tinyparser_mitsuba::Color reflectance = properties["reflectance"].getColor();

          glm::vec3 albedo = glm::vec3(reflectance.r, reflectance.g, reflectance.b);
          ConstColor col(albedo);
          texture_list.push_back(std::make_unique<ConstColor>(col));

          Lambertian l(texture_list[texture_list.size() - 1].get());
          list_materials.push_back(std::make_unique<Lambertian>(l));
        } else {
          fmt::println("plugin type {} is not supported.", plugin_type);
          return nullptr;
        }
      } else if (mat_obj->type() == tinyparser_mitsuba::OT_EMITTER) {
        if (plugin_type == "area") {
          auto properties = mat_obj->properties();
          tinyparser_mitsuba::Color light = properties["radiance"].getColor();

          glm::vec3 emit = glm::vec3(light.r, light.g, light.b);

          DiffuseLight d(emit);
          list_materials.push_back(std::make_unique<DiffuseLight>(d));
        } else {
          fmt::println("plugin type {} is not supported.", plugin_type);
          return nullptr;
        }
      } else {
        fmt::println("material type was neither OT_BSDF nor OT_EMITTER");
        return nullptr;
      }

      // add material to list and associate a name with the index
      if (bsdf_name != "") {
        name_to_mat[bsdf_name] = list_materials.size() - 1;
      } else {
        fmt::println("mat_name is empty");
      }

      return list_materials[list_materials.size() - 1].get();
    } else {
      fmt::println("plugin type twosided is not supported.");
      return nullptr;
    }
  }
}

bool set_scene_from_xml(const std::filesystem::path& path_file, integrator_data& integrator_data,
                        std::vector<std::unique_ptr<Surface>>& list_surfaces,
                        std::vector<std::unique_ptr<Material>>& list_materials,
                        std::vector<Surface*>& list_lights,
                        std::vector<std::unique_ptr<Mesh>>& list_meshes,
                        std::vector<std::unique_ptr<Texture>>& texture_list) {
  tinyparser_mitsuba::SceneLoader loader;

  // got the mitsuba file as scene object
  tinyparser_mitsuba::Scene scene = loader.loadFromFile(path_file.string());

  integrator_data.background_col = glm::vec3(0.f, 0.f, 0.f);

  std::unordered_map<std::string, size_t> name_to_mat;

  // convert scene objects to v-img objects
  // loop through once and get integrator_data and materials
  for (const auto& obj : scene.anonymousChildren()) {
    switch (obj->type()) {
      case tinyparser_mitsuba::OT_SENSOR: {
        auto properties = obj->properties();
        float h_fov = properties["fov"].getNumber();

        tinyparser_mitsuba::Transform to_world_m = properties["to_world"].getTransform();
        for (const auto& obj_child : obj->anonymousChildren()) {
          switch (obj_child->type()) {
            case tinyparser_mitsuba::OT_FILM: {
              auto child_p = obj_child->properties();
              int64_t width = child_p["width"].getInteger();
              int64_t height = child_p["height"].getInteger();

              // set up camera
              integrator_data.resolution = glm::ivec2(width, height);

              float hfov_rad = h_fov * M_PI / 180.f;
              float aspect_ratio = static_cast<float>(width) / static_cast<float>(height);

              float vfov = 2.f * atan(tan(hfov_rad / 2.f) * aspect_ratio) * (180.f / M_PI);

              glm::mat4 to_world;

              // glm is column major, tinyparser is row major
              int count = 0;
              for (size_t r = 0; r < 4; r++) {
                for (size_t c = 0; c < 4; c++) {
                  float m = 1.f;
                  // flip z and x axis for camera transform
                  if (c == 2 || c == 0) {
                    m = -1.f;
                  }

                  to_world[c][r] = to_world_m.matrix[count] * m;
                  ++count;
                }
              }

              integrator_data.camera = TLCam(to_world, integrator_data.resolution, vfov);
              break;
            }
            case tinyparser_mitsuba::OT_SAMPLER: {
              auto child_p = obj_child->properties();
              int64_t sample_count = child_p["sample_count"].getInteger();

              integrator_data.samples = sample_count;

              // assume always MIS for now
              integrator_data.func = integrator_func::mis;
              break;
            }
            default:
              break;
          }
        }
        break;
      }
      case tinyparser_mitsuba::OT_INTEGRATOR: {
        auto properties = obj->properties();
        int max_depth = properties["max_depth"].getInteger();
        integrator_data.depth = max_depth;
        break;
      }
      default:
        break;
    }
  }

  /*
   * loop through again to get the meshes and material
   */
  for (const auto& obj : scene.anonymousChildren()) {
    switch (obj->type()) {
      case tinyparser_mitsuba::OT_SHAPE: {
        auto obj_children = obj->anonymousChildren();
        auto properties = obj->properties();

        tinyparser_mitsuba::Transform to_world_m = properties["to_world"].getTransform();
        glm::mat4 transform;

        // glm is column major, tinyparser is row major
        int count = 0;
        for (size_t r = 0; r < 4; r++) {
          for (size_t c = 0; c < 4; c++) {
            transform[c][r] = to_world_m.matrix[count];
            ++count;
          }
        }

        // search for emitter
        bool emitter = false;
        Material* mat_ptr = nullptr;
        for (const auto& child : obj_children) {
          if (child->type() == tinyparser_mitsuba::OT_EMITTER) {
            mat_ptr = mat_index_from_obj(child, texture_list, list_materials, name_to_mat);
            emitter = true;
            break;
          }
        }

        // if emitter found skip mat if present
        if (emitter == false) {
          for (const auto& child : obj_children) {
            if (child->type() == tinyparser_mitsuba::OT_BSDF) {
              mat_ptr = mat_index_from_obj(child, texture_list, list_materials, name_to_mat);
              break;
            }
          }
        }

        if (mat_ptr == nullptr) {
          fmt::println("error in loading material");
          return false;
        }

        // load surface
        if (obj->pluginType() == "rectangle") {
          // create quad
          Quad q(glm::vec3(-1.f, -1.f, 0.f), glm::vec3(2.f, 0.f, 0.f), glm::vec3(0.f, 2.f, 0.f),
                 mat_ptr, transform);

          list_surfaces.push_back(std::make_unique<Quad>(q));

          if (emitter) {
            Surface* s_ptr = list_surfaces[list_surfaces.size() - 1].get();
            list_lights.push_back(s_ptr);
          }
        } else {
          fmt::println("shape plugin {} is not supported", obj->pluginType());
        }
        break;
      }
      default:
        break;
    }
  }

  return true;
}