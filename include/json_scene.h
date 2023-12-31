#pragma once

#include <integrators.h>
#include <tl_camera.h>

#include <nlohmann/json.hpp>
#include <string>
#include <unordered_map>

/*
    takes file path as input and sets the scene. Returns true if parsed,
    else returns false and print error
*/
bool set_scene_from_json(const std::string& path_file, integrator_data& integrator_data,
                         std::vector<std::unique_ptr<Surface>>& list_surfaces,
                         std::vector<std::unique_ptr<Material>>& list_materials);

// set integrator_data from json parsed
bool set_integrator_data(const nlohmann::json& json_settings, integrator_data& integrator_data);

bool set_list_of_materials(const nlohmann::json& json_settings,
                           std::vector<std::unique_ptr<Material>>& list_materials,
                           std::unordered_map<std::string, size_t>& name_to_index);

bool set_list_of_objects(const nlohmann::json& json_settings,
                         std::vector<std::unique_ptr<Surface>>& list_surfaces,
                         std::vector<std::unique_ptr<Material>>& list_materials,
                         const std::unordered_map<std::string, size_t>& name_to_index);