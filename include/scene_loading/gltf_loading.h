#pragma once

#include <geometry/mesh.h>
#include <integrators.h>
#include <texture/texture_RGB.h>

#include <nlohmann/json.hpp>

/*
    takes file path as input and sets the scene. Returns true if parsed,
    else returns false and print error
*/
bool set_scene_from_gltf(const std::filesystem::path& path_file, integrator_data& integrator_data,
                         std::vector<std::unique_ptr<Surface>>& list_surfaces,
                         std::vector<std::unique_ptr<Material>>& list_materials,
                         std::vector<Emitter*>& list_lights,
                         std::vector<std::unique_ptr<Mesh>>& list_meshes,
                         std::vector<std::unique_ptr<TextureRGB>>& texture_list,
                         const nlohmann::json& extra_settings);