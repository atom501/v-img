#pragma once

#include <bvh.h>
#include <geometry/surface.h>
#include <hit_utils.h>

#include <cstdint>
#include <string>

struct Mesh {
public:
  // list of triangle indices. Each triangle has 3 indices for each point
  std::vector<std::array<uint32_t, 3>> indices;
  std::vector<glm::vec3> vertices;  // coordinates of vertices loaded

  // optional attributes. may be empty vectors
  std::vector<glm::vec3> normals;    // normals of vertices loaded
  std::vector<glm::vec2> texcoords;  // uv of vertices loaded

  std::vector<glm::vec2> normal_coords;  // uv of normal map loaded

  Material* mat;

  Mesh(const std::vector<std::array<uint32_t, 3>>& indices, const std::vector<glm::vec3>& vertices,
       const std::vector<glm::vec3>& normals, const std::vector<glm::vec2>& texcoords,
       Material* mat)
      : indices(indices), vertices(vertices), normals(normals), texcoords(texcoords), mat(mat) {}

  Mesh(const std::vector<std::array<uint32_t, 3>>& indices, const std::vector<glm::vec3>& vertices,
       const std::vector<glm::vec3>& normals, const std::vector<glm::vec2>& texcoords,
       const std::vector<glm::vec2>& normal_coords, Material* mat)
      : indices(indices),
        vertices(vertices),
        normals(normals),
        texcoords(texcoords),
        normal_coords(normal_coords),
        mat(mat) {}

  ~Mesh() = default;
};

bool load_from_obj(std::string model_path_rel_file, std::vector<std::array<uint32_t, 3>>& indices,
                   std::vector<glm::vec3>& vertices, std::vector<glm::vec3>& normals,
                   std::vector<glm::vec2>& texcoords, const glm::mat4& transform);

Mesh create_quad_mesh(Material* mat_ptr, const glm::mat4& xform);

void add_tri_list_to_scene(std::vector<std::unique_ptr<Surface>>& list_surfaces, Mesh* mesh_ptr,
                           std::vector<Emitter*>& list_lights);
