#pragma once

#include <geometry/mesh.h>
#include <integrator_info.h>
#include <texture/texture_RG.h>
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
                         std::vector<std::unique_ptr<TextureRG>>& textureRG_list,
                         std::vector<std::unique_ptr<TextureRGB>>& texture_list,
                         const nlohmann::json& extra_settings);

template <class T> inline void hash_combine(std::size_t& seed, const T& v) {
  std::hash<T> hasher;
  seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

struct PairHash {
  template <class T1, class T2> std::size_t operator()(const std::pair<T1, T2>& p) const {
    auto h1 = std::hash<T1>{}(p.first);
    auto h2 = std::hash<T2>{}(p.second);

    size_t seed = 0;

    hash_combine(seed, h1);
    hash_combine(seed, h2);

    return seed;
  }
};