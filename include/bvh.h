#pragma once

#include <geometry/surface.h>
#include <hit_utils.h>

#include <cstdint>
#include <glm/vec3.hpp>
#include <memory>
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
  Bin() {};
  Bin(size_t obj_count, AABB aabb) : obj_count(obj_count), aabb(aabb) {}

  ~Bin() {};

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
  BVH() {};
  ~BVH() {};

  // input is list of bounding boxes of primitives and their centers
  static BVH build(const std::vector<AABB>& bboxes, const std::vector<glm::vec3>& centers,
                   const size_t num_bins);

  /*
   * If output type is set to std::optional<HitInfo>, this will perform same function as
   * bvh checking for hit and getting surface information. If output is set to uint32_t it will
   * be used to make the heatmap. Only the total number of primitives hit by the ray are needed.
   */
  template <typename T,
            std::enable_if_t<
                std::is_same_v<T, std::optional<HitInfo>> || std::is_same_v<T, uint32_t>, bool>
            = true>
  T hit(Ray& ray, std::vector<size_t>& thread_stack,
        const std::vector<std::unique_ptr<Surface>>& prims) const {
    T return_variable;

    if constexpr (std::is_same_v<T, std::optional<HitInfo>>) {
      return_variable = std::nullopt;
    } else if constexpr (std::is_same_v<T, uint32_t>) {
      return_variable = 0;
    }

    std::optional<float> bb_hit1;
    std::optional<float> bb_hit2;

    /*
     * check root node first. Since children are always hit checked before being pushed onto the
     * stack, no need to perform hit test again when popping the node (except root node)
     */
    auto& root_node = nodes[0];

    glm::vec3 ray_inv_dir;
    ray_inv_dir[0] = 1.0f / ray.dir[0];
    ray_inv_dir[1] = 1.0f / ray.dir[1];
    ray_inv_dir[2] = 1.0f / ray.dir[2];

    if (!root_node.aabb.intersect(ray, ray_inv_dir)) {
      if constexpr (std::is_same_v<T, std::optional<HitInfo>>) {
        return std::nullopt;
      } else if constexpr (std::is_same_v<T, uint32_t>) {
        return static_cast<uint32_t>(0);
      }
    }

    thread_stack.push_back(0);

    while (!thread_stack.empty()) {
      auto& node = nodes[thread_stack.back()];
      thread_stack.pop_back();

      // if leaf check all primitives in the leaf
      if (node.is_leaf()) {
        for (size_t i = 0; i < node.obj_count; ++i) {
          auto prim_index = obj_indices[node.first_index + i];

          // ray's tmax value will be changed if hit
          if constexpr (std::is_same_v<T, std::optional<HitInfo>>) {
            auto hit_temp = prims[prim_index]->hit(ray);
            // if ray hit the object replace the last hit_final
            if (hit_temp.has_value()) return_variable = hit_temp;
          } else if constexpr (std::is_same_v<T, uint32_t>) {
            prims[prim_index]->hit(ray);
            // increment whenever a hit test is done on a primitive
            ++return_variable;
          }
        }
      } else {
        // intersect with both children and push further bbox first
        size_t first_child = node.first_index;
        size_t sec_child = node.first_index + 1;

        bb_hit1 = nodes[first_child].aabb.intersect(ray, ray_inv_dir);
        bb_hit2 = nodes[sec_child].aabb.intersect(ray, ray_inv_dir);

        if (bb_hit2) {
          if (bb_hit1) {
            // both intersect. swap according to dist. push more distant bbox first
            if (bb_hit2.value() > bb_hit1.value()) {
              std::swap(first_child, sec_child);
            }
            thread_stack.push_back(first_child);
          }
          thread_stack.push_back(sec_child);
        } else if (bb_hit1) {
          thread_stack.push_back(first_child);
        }  // else don't push any child node
      }
    }

    return return_variable;
  }
};
