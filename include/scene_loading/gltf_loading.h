#pragma once

#include <geometry/mesh.h>
#include <integrators.h>
#include <texture.h>

/*
    takes file path as input and sets the scene. Returns true if parsed,
    else returns false and print error
*/
bool set_scene_from_gltf(const std::filesystem::path& path_file, integrator_data& integrator_data,
                         std::vector<std::unique_ptr<Surface>>& list_surfaces,
                         std::vector<std::unique_ptr<Material>>& list_materials,
                         std::vector<Emitter*>& list_lights,
                         std::vector<std::unique_ptr<Mesh>>& list_meshes,
                         std::vector<std::unique_ptr<Texture>>& texture_list);