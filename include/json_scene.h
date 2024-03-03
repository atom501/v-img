#pragma once

#include <integrators.h>
#include <tl_camera.h>

#include <glm/gtx/quaternion.hpp>
#include <nlohmann/json.hpp>
#include <string>
#include <unordered_map>

/*
    takes file path as input and sets the scene. Returns true if parsed,
    else returns false and print error
*/
bool set_scene_from_json(const std::string& path_file, integrator_data& integrator_data,
                         std::vector<std::unique_ptr<Surface>>& list_surfaces,
                         std::vector<std::unique_ptr<Material>>& list_materials,
                         std::vector<Surface*>& list_lights);

// set integrator_data from json parsed
bool set_integrator_data(const nlohmann::json& json_settings, integrator_data& integrator_data);

bool set_list_of_materials(const nlohmann::json& json_settings,
                           std::vector<std::unique_ptr<Material>>& list_materials,
                           std::unordered_map<std::string, size_t>& name_to_index);

bool set_list_of_objects(const nlohmann::json& json_settings,
                         std::vector<std::unique_ptr<Surface>>& list_surfaces,
                         std::vector<std::unique_ptr<Material>>& list_materials,
                         std::vector<Surface*>& list_lights,
                         const std::unordered_map<std::string, size_t>& name_to_index);

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