#pragma once

#include <bvh.h>
#include <geometry/surface.h>
#include <hit_utils.h>

#include <cstdint>
#include <string>

namespace MeshConsts {
  constexpr uint8_t no_uv = 255;
}

struct Mesh {
public:
  // list of triangle indices. Each triangle has 3 indices for each point
  std::vector<std::array<uint32_t, 3>> indices;
  std::vector<glm::vec3> vertices;  // coordinates of vertices loaded

  // optional attributes. may be empty vectors
  std::vector<glm::vec3> normals;  // normals of vertices loaded

  // all uv attributes will point to index in texcoords. to avoid duplication
  std::vector<std::vector<glm::vec2>> texcoords;

  uint8_t color_tex_uv = MeshConsts::no_uv;
  uint8_t normal_tex_uv = MeshConsts::no_uv;
  uint8_t metallic_roughness_tex_uv = MeshConsts::no_uv;

  Material* mat;

  // for backwards compatibility. remove later
  Mesh(const std::vector<std::array<uint32_t, 3>>& indices, const std::vector<glm::vec3>& vertices,
       const std::vector<glm::vec3>& normals, const std::vector<glm::vec2>& in_texcoords,
       Material* mat)
      : indices(indices), vertices(vertices), normals(normals), mat(mat) {
    if (!in_texcoords.empty()) {
      Mesh::texcoords.push_back(in_texcoords);
      color_tex_uv = 0;
    }
  }

  Mesh(const std::vector<std::array<uint32_t, 3>>& indices, const std::vector<glm::vec3>& vertices,
       const std::vector<glm::vec3>& normals, const std::vector<std::vector<glm::vec2>>& texcoords,
       Material* mat, uint8_t color_tex_uv, uint8_t normal_tex_uv,
       uint8_t metallic_roughness_tex_uv)
      : indices(indices),
        vertices(vertices),
        normals(normals),
        texcoords(texcoords),
        mat(mat),
        color_tex_uv(color_tex_uv),
        normal_tex_uv(normal_tex_uv),
        metallic_roughness_tex_uv(metallic_roughness_tex_uv) {}

  ~Mesh() = default;
};

bool load_from_obj(std::string model_path_rel_file, std::vector<std::array<uint32_t, 3>>& indices,
                   std::vector<glm::vec3>& vertices, std::vector<glm::vec3>& normals,
                   std::vector<glm::vec2>& texcoords, const glm::mat4& transform);

Mesh create_quad_mesh(Material* mat_ptr, const glm::mat4& xform);

void add_tri_list_to_scene(std::vector<std::unique_ptr<Surface>>& list_surfaces, Mesh* mesh_ptr,
                           std::vector<Emitter*>& list_lights);
