#pragma once

#include <geometry/emitters.h>
#include <geometry/surface.h>

#include <cstdint>
#include <glm/gtx/norm.hpp>

struct Mesh;

static inline float difference_of_products(float a, float b, float c, float d) {
  float cd = c * d;
  float differenceOfProducts = std::fma(a, b, -cd);
  return differenceOfProducts;
}

static inline double difference_of_products_double(float a, float b, float c, float d) {
  double cd = c * d;
  double differenceOfProducts = std::fmal(a, b, -cd);
  return differenceOfProducts;
}

static inline int max_componenet_index(const glm::vec3& dir) {
  glm::vec3 abs_val = glm::abs(dir);

  int index = 0;
  float max_val = abs_val.x;

  if (abs_val.y > max_val) {
    index = 1;
    max_val = abs_val.y;
  }

  if (abs_val.z > max_val) {
    index = 2;
    max_val = abs_val.z;
  }

  return index;
}

static inline glm::vec3 permute_points(const glm::vec3& orig_vec, const glm::ivec3& indexes) {
  return glm::vec3(orig_vec[indexes.x], orig_vec[indexes.y], orig_vec[indexes.z]);
}

class Triangle : public Surface, public Emitter {
public:
  Mesh* obj_mesh = nullptr;
  uint32_t tri_index;  // triangle number in Mesh

public:
  Triangle(Mesh* obj_mesh, uint32_t index) : obj_mesh(obj_mesh), tri_index(index) {}

  ~Triangle() = default;

  std::optional<ForHitInfo> hit_surface(Ray& r) const override;
  bool hit_check(Ray& r) const override;
  HitInfo hit_info(const Ray& r, const ForHitInfo& pre_calc) const override;

  AABB bounds() const override;
  glm::vec3 get_center() const override;

  std::pair<glm::vec3, EmitterInfo> sample(const glm::vec3& look_from,
                                           pcg32_random_t& pcg_rng) const override;

  float surf_pdf(const glm::vec3& look_from, const glm::vec3& look_at,
                 const glm::vec3& dir) const override;
};