#pragma once

#include <geometry/surface.h>
#include <hit_utils.h>

#include <cstdint>
#include <glm/vec3.hpp>
#include <vector>

struct BVHNode {
  AABB aabb;
  uint32_t first_index;  // index of next node or list of objects in leaf
  uint32_t obj_count;    // number of objects in leaf

  // returns true when is leaf
  bool is_leaf() const { return obj_count != 0; };
};

class Bin {
public:
  AABB aabb;
  size_t obj_count;

public:
  Bin(){};
  Bin(size_t obj_count, AABB aabb) : obj_count(obj_count), aabb(aabb) {}

  ~Bin(){};

  float cost() { return aabb.half_SA() * obj_count; }
};

struct Split {
  size_t bin_split;  // right bin to use for splitting
  float cost;
  uint8_t axis;
};

class BVH {
public:
  std::vector<BVHNode> nodes;
  std::vector<size_t> obj_indices;  // indices pointing to original object list

public:
  BVH(){};
  ~BVH(){};

  // input is list of bounding boxes of primitives and their centers
  static BVH build(const std::vector<AABB>& bboxes, const std::vector<glm::vec3>& centers,
                   const size_t num_bins);

  std::optional<HitInfo> hit(Ray& ray, const std::vector<Surface*>& prims) const;
};
