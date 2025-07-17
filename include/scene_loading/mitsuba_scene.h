#pragma once

#include <geometry/mesh.h>
#include <integrators.h>
#include <scene_loading/tinyparser-mitsuba.h>
#include <texture.h>
#include <tl_camera.h>

#include <glm/gtx/quaternion.hpp>
#include <nlohmann/json.hpp>
#include <string>
#include <unordered_map>

/*
    takes file path as input and sets the scene. Returns true if parsed,
    else returns false and print error
*/
bool set_scene_from_xml(const std::filesystem::path& path_file, integrator_data& integrator_data,
                        std::vector<std::unique_ptr<Surface>>& list_surfaces,
                        std::vector<std::unique_ptr<Material>>& list_materials,
                        std::vector<Emitter*>& list_lights,
                        std::vector<std::unique_ptr<Mesh>>& list_meshes,
                        std::vector<std::unique_ptr<Texture>>& texture_list);

float hfov_deg_to_vfov_deg(float h_fov_deg, int64_t width, int64_t height);