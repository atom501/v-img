#pragma once

#include <bvh.h>
#include <geometry/surface.h>
#include <hit_utils.h>

#include <cstdint>

struct Mesh {
public:
  std::vector<glm::vec3> vertices;   // coordinates of vertices loaded
  std::vector<uint32_t> tri_vertex;  // for a triangle vertex, gives it's index in vector of
                                     // vertices. [3*i, 3*i+1, 3*i+2] for i th triangle

  // optional additional info per point
  std::vector<uint32_t> tri_normal;  // same as tri_vertex but for normals
  std::vector<glm::vec3> normals;    // normals of vertices loaded

  std::vector<int> tri_uv;           // same as tri_vertex but for uvs
  std::vector<glm::vec2> texcoords;  // uv of vertices loaded

  Mesh(const std::vector<glm::vec3>& vertices, const std::vector<uint32_t>& tri_vertex,
       const std::vector<glm::vec3>& normals, const std::vector<uint32_t>& tri_normal,
       const std::vector<glm::vec2>& texcoords, const std::vector<int>& tri_uv)
      : vertices(vertices),
        tri_vertex(tri_vertex),
        normals(normals),
        tri_normal(tri_normal),
        texcoords(texcoords),
        tri_uv(tri_uv) {}

  ~Mesh() = default;
};
