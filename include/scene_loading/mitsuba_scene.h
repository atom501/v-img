#pragma once
/*
    takes file path as input and sets the scene. Returns true if parsed,
    else returns false and print error
*/
bool set_scene_from_mitsuba_xml(const std::filesystem::path& path_file,
                                integrator_data& integrator_data,
                                std::vector<std::unique_ptr<Surface>>& list_surfaces,
                                std::vector<std::unique_ptr<Material>>& list_materials,
                                std::vector<Emitter*>& list_lights,
                                std::vector<std::unique_ptr<Mesh>>& list_meshes,
                                std::vector<std::unique_ptr<TextureRG>>& textureRG_list,
                                std::vector<std::unique_ptr<TextureRGB>>& texture_list);

float hfov_deg_to_vfov_deg(float h_fov_deg, int64_t width, int64_t height);