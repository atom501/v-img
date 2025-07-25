#include <background.h>
#include <geometry/mesh.h>
#include <geometry/quads.h>
#include <geometry/sphere.h>
#include <geometry/triangle.h>
#include <material/diffuse_light.h>
#include <material/lambertian.h>
#include <scene_loading/mitsuba_scene.h>
#include <scene_loading/serialized_file.h>
#include <tinyexr.h>

#include <numbers>

float hfov_deg_to_vfov_deg(float h_fov_deg, int64_t width, int64_t height) {
  float hfov_rad = h_fov_deg * std::numbers::pi / 180.f;
  float aspect_ratio = static_cast<float>(width) / static_cast<float>(height);

  float vfov_deg = 2.f * atan(tan(hfov_rad / 2.f) * aspect_ratio) * (180.f / std::numbers::pi);

  return vfov_deg;
}

void cube_mesh(std::vector<glm::vec3>& vertices, std::vector<glm::vec3>& normals,
               std::vector<glm::vec2>& texcoords, std::vector<uint32_t>& tri_vertex,
               std::vector<uint32_t>& tri_normal, std::vector<int>& tri_uv) {
  // data for mesh object
  vertices = {glm::vec3(1.f, -1.f, -1.f),  glm::vec3(1.f, -1.f, 1.f), glm::vec3(-1.f, -1.f, 1.f),
              glm::vec3(-1.f, -1.f, -1.f), glm::vec3(1.f, 1.f, -1.f), glm::vec3(1.f, 1.f, 1.f),
              glm::vec3(-1.f, 1.f, 1.f),   glm::vec3(-1.f, 1.f, -1.f)};

  normals = {glm::vec3(0.f, -1.f, 0.f), glm::vec3(0.f, 1.f, 0.f),  glm::vec3(1.f, 0.f, 0.f),
             glm::vec3(0.f, 0.f, 1.f),  glm::vec3(-1.f, 0.f, 0.f), glm::vec3(0.f, 0.f, -1.f)};

  texcoords = {
      glm::vec2(1.000000, 0.333333), glm::vec2(1.000000, 0.666667), glm::vec2(0.666667, 0.666667),
      glm::vec2(0.666667, 0.333333), glm::vec2(0.666667, 0.000000), glm::vec2(0.000000, 0.333333),
      glm::vec2(0.000000, 0.000000), glm::vec2(0.333333, 0.000000), glm::vec2(0.333333, 1.000000),
      glm::vec2(0.000000, 1.000000), glm::vec2(0.000000, 0.666667), glm::vec2(0.333333, 0.333333),
      glm::vec2(0.333333, 0.666667), glm::vec2(1.000000, 0.000000),
  };

  tri_vertex = {1, 2, 3, 7, 6, 5, 4, 5, 1, 5, 6, 2, 2, 6, 7, 0, 3, 7,
                0, 1, 3, 4, 7, 5, 0, 4, 1, 1, 5, 2, 3, 2, 7, 4, 0, 7};
  tri_normal = {0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4, 5, 5, 5,
                0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4, 5, 5, 5};
  tri_uv = {0, 1, 2, 0,  3, 4, 5,  6, 7, 7,  4, 3, 8,  9, 10, 11, 12, 10,
            3, 0, 2, 13, 0, 4, 11, 5, 7, 11, 7, 3, 12, 8, 10, 5,  11, 10};
}

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
          bool result;
          tinyparser_mitsuba::Color reflectance
              = properties["reflectance"].getColor(tinyparser_mitsuba::Color(0, 0, 0), &result);

          if (result) {
            glm::vec3 albedo = glm::vec3(reflectance.r, reflectance.g, reflectance.b);
            ConstColor col(albedo);
            texture_list.push_back(std::make_unique<ConstColor>(col));

            Lambertian l(texture_list[texture_list.size() - 1].get());
            list_materials.push_back(std::make_unique<Lambertian>(l));
          } else {
            auto named_children = mat_obj->namedChildren();
            bool found = false;

            for (auto& [_, value] : named_children) {
              if (value->type() == tinyparser_mitsuba::OT_TEXTURE) {
                if (value->pluginType() == "checkerboard") {
                  auto child_p = value->properties();
                  tinyparser_mitsuba::Color c0 = child_p["color0"].getColor();
                  tinyparser_mitsuba::Color c1 = child_p["color1"].getColor();
                  int t_width = child_p["uscale"].getNumber();
                  int t_height = child_p["vscale"].getNumber();

                  Checkerboard ch(t_width * 2, t_height * 2, glm::vec3(c0.r, c0.g, c0.b),
                                  glm::vec3(c1.r, c1.g, c1.b));
                  texture_list.push_back(std::make_unique<Checkerboard>(ch));

                  Lambertian l(texture_list[texture_list.size() - 1].get());
                  list_materials.push_back(std::make_unique<Lambertian>(l));
                  found = true;
                  break;
                }
              }
            }

            if (!found) {
              auto mat_c = mat_obj->anonymousChildren();
              for (auto& value : mat_c) {
                if (value->type() == tinyparser_mitsuba::OT_TEXTURE) {
                  if (value->pluginType() == "checkerboard") {
                    auto child_p = value->properties();
                    tinyparser_mitsuba::Color c0 = child_p["color0"].getColor();
                    tinyparser_mitsuba::Color c1 = child_p["color1"].getColor();
                    int t_width = child_p["uscale"].getNumber();
                    int t_height = child_p["vscale"].getNumber();

                    Checkerboard ch(t_width, t_height, glm::vec3(c0.r, c0.g, c0.b),
                                    glm::vec3(c1.r, c1.g, c1.b));
                    texture_list.push_back(std::make_unique<Checkerboard>(ch));

                    Lambertian l(texture_list[texture_list.size() - 1].get());
                    list_materials.push_back(std::make_unique<Lambertian>(l));
                    break;
                  }
                }
              }
            }
          }
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
        // fmt::println("mat_name is empty");
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
                        std::vector<Emitter*>& list_lights,
                        std::vector<std::unique_ptr<Mesh>>& list_meshes,
                        std::vector<std::unique_ptr<Texture>>& texture_list) {
  tinyparser_mitsuba::SceneLoader loader;

  // got the mitsuba file as scene object
  tinyparser_mitsuba::Scene scene = loader.loadFromFile(path_file.string());

  integrator_data.background = std::make_unique<ConstBackground>(ConstBackground(glm::vec3(0)));

  std::unordered_map<std::string, size_t> name_to_mat;

  // convert scene objects to v-img objects
  // loop through once and get integrator_data and envmap
  for (const auto& obj : scene.anonymousChildren()) {
    switch (obj->type()) {
      case tinyparser_mitsuba::OT_SENSOR: {
        auto properties = obj->properties();
        float fov_deg = properties["fov"].getNumber();
        std::string fov_axis = properties["fov_axis"].getString("x");

        tinyparser_mitsuba::Transform to_world_m = properties["to_world"].getTransform();
        for (const auto& obj_child : obj->anonymousChildren()) {
          switch (obj_child->type()) {
            case tinyparser_mitsuba::OT_FILM: {
              auto child_p = obj_child->properties();
              int64_t width = child_p["width"].getInteger();
              int64_t height = child_p["height"].getInteger();

              // set up camera
              integrator_data.resolution = glm::ivec2(width, height);
              float vfov;

              if (fov_axis == "x") {
                vfov = hfov_deg_to_vfov_deg(fov_deg, width, height);
              } else if (fov_axis == "y") {
                vfov = fov_deg;
              } else if (fov_axis == "smaller") {
                if (width < height)
                  vfov = hfov_deg_to_vfov_deg(fov_deg, width, height);
                else
                  vfov = fov_deg;
              } else if (fov_axis == "larger") {
                if (width > height)
                  vfov = hfov_deg_to_vfov_deg(fov_deg, width, height);
                else
                  vfov = fov_deg;
              } else {
                fmt::println("fov axis {} is unknown. defaulting to x axis", fov_axis);
                vfov = hfov_deg_to_vfov_deg(fov_deg, width, height);
              }

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

        if (obj->pluginType() == "path") {
          integrator_data.func = integrator_func::mis;
        } else if (obj->pluginType() == "mat") {
          integrator_data.func = integrator_func::material;
        } else if (obj->pluginType() == "s_normal") {
          integrator_data.func = integrator_func::s_normal;
        } else if (obj->pluginType() == "g_normal") {
          integrator_data.func = integrator_func::g_normal;
        } else {
          fmt::println("Unknown integrator {}. Default to MIS", obj->pluginType());
          integrator_data.func = integrator_func::mis;
        }
        break;
      }
      case tinyparser_mitsuba::OT_EMITTER: {
        // check if envmap. can be constant or image
        if (obj->pluginType() == "envmap") {
          auto p = obj->properties();

          std::filesystem::path env_file = p["filename"].getString();
          float radiance_scale = p["scale"].getNumber(1.f);

          tinyparser_mitsuba::Transform to_world_m = p["to_world"].getTransform();
          glm::mat4 transform;

          // glm is column major, tinyparser is row major
          int count = 0;
          for (size_t r = 0; r < 4; r++) {
            for (size_t c = 0; c < 4; c++) {
              transform[c][r] = to_world_m.matrix[count];
              ++count;
            }
          }

          // read exr file
          std::string env_type = env_file.extension().generic_string();
          if (env_type == ".exr") {
            // read exr with tinyexr
            float* out;  // width * height * RGBA
            int width;
            int height;
            const char* err = nullptr;

            std::filesystem::path scene_filename = path_file;
            const auto env_path_rel_file = scene_filename.remove_filename() / env_file;

            int ret
                = LoadEXR(&out, &width, &height, env_path_rel_file.generic_string().c_str(), &err);
            std::vector<glm::vec3> image;

            if (ret != TINYEXR_SUCCESS) {
              if (err) {
                fprintf(stderr, "ERR : %s\n", err);
                FreeEXRErrorMessage(err);  // release memory of error message.
              }
            } else {
              // copy image data to vector. ignore alpha channel for now
              image = std::vector<glm::vec3>(width * height);

              for (size_t i = 0; i < width * height; i++) {
                image[i] = glm::vec3(out[i * 4], out[i * 4 + 1], out[i * 4 + 2]);
              }

              free(out);  // release memory of image data

              // set env map
              integrator_data.background = std::make_unique<EnvMap>(
                  EnvMap(width, height, image, glm::inverse(transform), transform, radiance_scale));

              list_lights.push_back(static_cast<EnvMap*>(integrator_data.background.get()));
            }
          } else {
            fmt::println("env map file type {} is not supported", env_type);
          }

        } else if (obj->pluginType() == "constant") {
          auto p = obj->properties();
          tinyparser_mitsuba::Color c = p["radiance"].getColor();

          glm::vec3 const_col(c.r, c.g, c.b);
          integrator_data.background
              = std::make_unique<ConstBackground>(ConstBackground(const_col));
          list_lights.push_back(static_cast<ConstBackground*>(integrator_data.background.get()));
        }
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

        Material* mat_ptr = nullptr;

        // loop over named children if present
        // search for emitter
        auto named_children = obj->namedChildren();
        for (auto& [_, value] : named_children) {
          if (value->type() == tinyparser_mitsuba::OT_EMITTER) {
            mat_ptr = mat_index_from_obj(value, texture_list, list_materials, name_to_mat);
            break;
          }
        }

        // if emitter found skip mat if present
        if (!mat_ptr) {
          for (auto& [_, value] : named_children) {
            if (value->type() == tinyparser_mitsuba::OT_BSDF) {
              mat_ptr = mat_index_from_obj(value, texture_list, list_materials, name_to_mat);
              break;
            }
          }
        }

        // loop over unnamed children if present. skip if named was already found
        // search for emitter
        if (!mat_ptr) {
          for (const auto& child : obj_children) {
            if (child->type() == tinyparser_mitsuba::OT_EMITTER) {
              mat_ptr = mat_index_from_obj(child, texture_list, list_materials, name_to_mat);
              break;
            }
          }
        }

        // if emitter found skip mat if present
        if (!mat_ptr) {
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

          if (mat_ptr->is_emissive()) {
            Quad* s_ptr = static_cast<Quad*>(list_surfaces[list_surfaces.size() - 1].get());
            list_lights.push_back(s_ptr);
          }
        } else if (obj->pluginType() == "cube") {
          // manually set cube mesh
          std::vector<glm::vec3> vertices;
          std::vector<glm::vec3> normals;
          std::vector<glm::vec2> texcoords;

          std::vector<uint32_t> tri_vertex;
          std::vector<uint32_t> tri_normal;
          std::vector<int> tri_uv;

          cube_mesh(vertices, normals, texcoords, tri_vertex, tri_normal, tri_uv);

          // transform vertices
          for (glm::vec3& vec : vertices) {
            glm::vec4 result = transform * glm::vec4(vec, 1);
            result /= result.w;
            vec = result;
          }

          // transform normals
          const glm::mat4 normal_xform = glm::transpose(glm::inverse(transform));
          for (glm::vec3& normal : normals) {
            glm::vec4 result = normal_xform * glm::vec4(normal, 0);
            normal = result;
          }

          vertices.shrink_to_fit();
          normals.shrink_to_fit();
          texcoords.shrink_to_fit();

          tri_vertex.shrink_to_fit();
          tri_normal.shrink_to_fit();
          tri_uv.shrink_to_fit();

          // make mesh
          auto tri_mesh = Mesh(vertices, tri_vertex, normals, tri_normal, texcoords, tri_uv);
          list_meshes.push_back(std::make_unique<Mesh>(tri_mesh));

          int num_tri = tri_vertex.size() / 3;

          // add to list of surfaces
          for (size_t i = 0; i < num_tri; i++) {
            auto tri = Triangle(list_meshes[list_meshes.size() - 1].get(), i, mat_ptr);
            list_surfaces.push_back(std::make_unique<Triangle>(tri));
          }

          // add to list of lights if needed
          size_t rev_count_index = list_surfaces.size() - 1;

          if (mat_ptr->is_emissive()) {
            for (size_t i = rev_count_index; i > (rev_count_index - num_tri); i--) {
              Triangle* s_ptr = static_cast<Triangle*>(list_surfaces[i].get());
              list_lights.push_back(s_ptr);
            }
          }
        } else if (obj->pluginType() == "serialized") {
          std::string s_file = properties["filename"].getString();
          std::filesystem::path model_filename = properties["filename"].getString();
          std::filesystem::path scene_filename = path_file;

          const auto model_path_rel_file = scene_filename.remove_filename() / model_filename;
          int shape_index = properties["shape_index"].getInteger();

          Mesh serialized_mesh = read_serialized_file(model_path_rel_file, shape_index, transform);

          list_meshes.push_back(std::make_unique<Mesh>(serialized_mesh));

          int num_tri = serialized_mesh.tri_vertex.size() / 3;

          // add to list of surfaces
          for (size_t i = 0; i < num_tri; i++) {
            auto tri = Triangle(list_meshes[list_meshes.size() - 1].get(), i, mat_ptr);
            list_surfaces.push_back(std::make_unique<Triangle>(tri));
          }

          // add to list of lights if needed
          size_t rev_count_index = list_surfaces.size() - 1;

          if (mat_ptr->is_emissive()) {
            for (size_t i = rev_count_index; i > (rev_count_index - num_tri); i--) {
              Triangle* s_ptr = static_cast<Triangle*>(list_surfaces[i].get());
              list_lights.push_back(s_ptr);
            }
          }
        } else if (obj->pluginType() == "sphere") {
          fmt::println("sphere");
          float radius = properties["radius"].getNumber(1.0f);
          tinyparser_mitsuba::Vector center_p = properties["center"].getVector();

          glm::vec3 center(center_p.x, center_p.y, center_p.z);

          // create sphere
          Sphere s(center, radius, mat_ptr);

          list_surfaces.push_back(std::make_unique<Sphere>(s));

          if (mat_ptr->is_emissive()) {
            Sphere* s_ptr = static_cast<Sphere*>(list_surfaces[list_surfaces.size() - 1].get());
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