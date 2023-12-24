#pragma once

#include <integrators.h>
#include <tl_camera.h>

#include <nlohmann/json.hpp>
#include <string>

/*
    takes file path as input and sets the scene. Returns true if parsed,
    else returns false and print error
*/
bool set_scene_from_json(const std::string& path_file, integrator_data& integrator_data);

// set integrator_data from json parsed
bool set_integrator_data(const nlohmann::json& json_settings, integrator_data& integrator_data);