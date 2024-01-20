#pragma once

#include <bvh.h>
#include <geometry/surface.h>
#include <hit_utils.h>

#include <cstdint>

class Mesh : public Surface {
public:
  uint32_t num_triangles = 0;
  std::vector<glm::vec3> vertex;     // coordinates of vertices loaded
  std::vector<uint32_t> tri_vertex;  // for a triangle vertex gives it's index in point vector.
                                     // [i, i+1, i+2] for i th triangle
  std::vector<std::unique_ptr<Surface>> triangles;  // stored as Surface class as bvh uses it

  AABB bbox;    // bounding box for the mesh
  BVH tri_bvh;  // bvh of all triangles in the mesh

  // optional additional info per point
  std::vector<glm::vec3> point_normal;  // normal of vertices loaded

public:
  Mesh(){};
  ~Mesh(){};
};

class Triangle : public Surface {
public:
  Mesh* obj_mesh = nullptr;
  uint32_t tri_index;  // triangle number in Mesh

public:
  Triangle(Mesh* mesh_ptr, uint32_t index, Material* mat_ptr)
      : obj_mesh(mesh_ptr), tri_index(index), Surface(mat_ptr) {}

  void transform(const glm::mat4& xform) override;
  std::optional<HitInfo> hit(Ray& r) const override;
  AABB bounds() const override;
  glm::vec3 get_center() const override;

  glm::vec3 sample(const glm::vec3& look_from, EmitterInfo& emit_info, float rand1,
                   float rand2) const override;
  float pdf(const glm::vec3& look_from, const glm::vec3& look_at,
            const glm::vec3& dir) const override;

  ~Triangle(){};
};
