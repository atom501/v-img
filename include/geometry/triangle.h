#pragma once

#include <geometry/mesh.h>
#include <geometry/surface.h>
#include <hit_utils.h>

#include <cstdint>

class Triangle : public Surface {
public:
  Mesh* obj_mesh = nullptr;
  uint32_t tri_index;  // triangle number in Mesh

public:
  Triangle(Mesh* mesh_ptr, uint32_t index, Material* mat_ptr)
      : obj_mesh(mesh_ptr), tri_index(index), Surface(mat_ptr) {}

  std::optional<HitInfo> hit(Ray& r) const override;
  AABB bounds() const override;
  glm::vec3 get_center() const override;

  glm::vec3 sample(const glm::vec3& look_from, EmitterInfo& emit_info, float rand1,
                   float rand2) const override;
  float pdf(const glm::vec3& look_from, const glm::vec3& look_at,
            const glm::vec3& dir) const override;

  ~Triangle() {}
};