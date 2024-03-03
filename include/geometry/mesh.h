#pragma once

#include <bvh.h>
#include <geometry/surface.h>
#include <hit_utils.h>

#include <cstdint>

class Mesh : public Surface {
public:
  std::vector<glm::vec3> vertices;   // coordinates of vertices loaded
  std::vector<uint32_t> tri_vertex;  // for a triangle vertex, gives it's index in vector of
                                     // vertices. [3*i, 3*i+1, 3*i+2] for i th triangle
  std::vector<std::unique_ptr<Surface>> triangles;  // stored as Surface class as bvh uses it

  AABB bbox;    // bounding box for the mesh
  BVH tri_bvh;  // bvh of all triangles in the mesh

  // optional additional info per point
  std::vector<uint32_t> tri_normal;  // same as tri_vertex but for normals
  std::vector<glm::vec3> normals;    // normals of vertices loaded

public:
  Mesh(const std::vector<glm::vec3>& vertices, const std::vector<uint32_t>& tri_vertex,
       const std::vector<glm::vec3>& normals, const std::vector<uint32_t>& tri_normal,
       Material* mat_ptr, const AABB& bbox)
      : vertices(vertices),
        tri_vertex(tri_vertex),
        normals(normals),
        tri_normal(tri_normal),
        bbox(bbox),
        Surface(mat_ptr) {
    // init the vector of triangle surfaces
    

    // iterate through triangles
  }

  std::optional<HitInfo> hit(Ray& r) const override;
  AABB bounds() const override;
  glm::vec3 get_center() const override;

  glm::vec3 sample(const glm::vec3& look_from, EmitterInfo& emit_info, float rand1,
                   float rand2) const override;
  float pdf(const glm::vec3& look_from, const glm::vec3& look_at,
            const glm::vec3& dir) const override;

  ~Mesh() {}
};
