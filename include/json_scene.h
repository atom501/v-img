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
                         std::vector<Surface*>& list_lights,
                         std::vector<std::unique_ptr<Mesh>>& list_meshes);

// set integrator_data from json parsed
bool set_integrator_data(const nlohmann::json& json_settings, integrator_data& integrator_data);

bool set_list_of_materials(const nlohmann::json& json_settings,
                           std::vector<std::unique_ptr<Material>>& list_materials,
                           std::unordered_map<std::string, size_t>& name_to_index);

bool set_list_of_objects(const nlohmann::json& json_settings,
                         std::vector<std::unique_ptr<Surface>>& list_surfaces,
                         std::vector<std::unique_ptr<Material>>& list_materials,
                         std::vector<Surface*>& list_lights,
                         const std::unordered_map<std::string, size_t>& name_to_index,
                         std::vector<std::unique_ptr<Mesh>>& list_meshes);
