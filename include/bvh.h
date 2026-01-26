#pragma once

#include <geometry/surface.h>
#include <hit_utils.h>

#ifdef __AVX2__
#include <simd_hit.h>
#endif

#include <array>
#include <cstdint>
#include <glm/vec3.hpp>
#include <memory>
#include <span>
#include <vector>

namespace BVHConst {
  constexpr float intersection_cost = 1.f;
  constexpr float traversal_cost = 0.5f;
}

struct BVHNode {
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
  Bin() = default;
  Bin(size_t obj_count, AABB aabb) : obj_count(obj_count), aabb(aabb) {}

  ~Bin() = default;

  float cost() { return aabb.surface_area() * obj_count; }
};

struct Split {
  size_t bin_split;  // right bin to use for splitting
  float cost;
  uint8_t axis;
  AABB left_aabb;
};

class BVH {
public:
  std::vector<BVHNode> nodes;
  std::vector<std::array<float, 3>> BB_mins_maxes;
  std::vector<size_t> obj_indices;  // indices pointing to original object list

  uint32_t max_depth;

public:
  BVH() = default;
  ~BVH() = default;

  // input is list of bounding boxes of primitives and their centers
  static BVH build_bin_bvh(const std::vector<AABB>& bboxes, const std::vector<glm::vec3>& centers,
                           const size_t num_bins);

  static BVH build_bonsai_bvh(const std::vector<AABB>& bboxes, bool prune,
                              const uint16_t max_node_primes);

  static BVH build_sweep_bvh(const std::vector<AABB>& bboxes, const std::vector<glm::vec3>& centers,
                             std::span<size_t> prim_indices, const uint16_t max_node_primes);

  bool occlude(Ray& ray, std::vector<size_t>& thread_stack,
               const std::vector<std::unique_ptr<Surface>>& prims) const {
    return hit<bool>(ray, thread_stack, prims);
  }

  /*
   * If output type is set to std::optional<HitInfo>, this will perform same function as
   * bvh checking for hit and getting surface information. If output is set to uint32_t it will
   * be used to make the heatmap. Only the total number of primitives hit by the ray are needed.
   */
  template <typename T,
            std::enable_if_t<std::is_same_v<T, std::optional<HitInfo>>
                                 || std::is_same_v<T, uint32_t> || std::is_same_v<T, bool>,
                             bool>
            = true>
  T hit(Ray& ray, std::vector<size_t>& thread_stack,
        const std::vector<std::unique_ptr<Surface>>& prims) const {
    T return_variable;
    std::optional<ForHitInfo> intermediate_hit = std::nullopt;

    if constexpr (std::is_same_v<T, std::optional<HitInfo>>) {
      return_variable = std::nullopt;
    } else if constexpr (std::is_same_v<T, uint32_t>) {
      return_variable = 0;
    } else if constexpr (std::is_same_v<T, bool>) {
      return_variable = false;
    }

    if (BVH::nodes.size() == 0) {
      return return_variable;
    }

    /*
     * check root node first. Since children are always hit checked before being pushed onto the
     * stack, no need to perform hit test again when popping the node (except root node)
     */
    auto& root_node = nodes[0];

#ifdef __AVX2__
    __m256 ray_dir_inv
        = _mm256_set_ps(1.f, ray.dir.z, ray.dir.y, ray.dir.x, 1.f, ray.dir.z, ray.dir.y, ray.dir.x);
    ray_dir_inv = _mm256_rcp_ps(ray_dir_inv);

    __m256 ray_o = _mm256_set_ps(0.f, ray.o.z, ray.o.y, ray.o.x, 0.f, ray.o.z, ray.o.y, ray.o.x);

    float root_hit
        = ray_1aabb_slab(BB_mins_maxes[0].data(), BB_mins_maxes[2].data(), ray_o, ray_dir_inv, ray);
#else
    glm::vec3 ray_inv_dir;
    ray_inv_dir[0] = 1.0f / ray.dir[0];
    ray_inv_dir[1] = 1.0f / ray.dir[1];
    ray_inv_dir[2] = 1.0f / ray.dir[2];

    float root_hit
        = slab_intersect_aabb_array(ray, ray_inv_dir, BB_mins_maxes[0], BB_mins_maxes[2]);
#endif

    if (std::isinf(root_hit)) {
      if constexpr (std::is_same_v<T, std::optional<HitInfo>>) {
        return std::nullopt;
      } else if constexpr (std::is_same_v<T, uint32_t>) {
        return static_cast<uint32_t>(0);
      } else if constexpr (std::is_same_v<T, bool>) {
        return false;
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
            const auto hit_temp = prims[prim_index]->hit_surface(ray);
            // if ray hit the object replace the last hit_final
            if (hit_temp.has_value()) intermediate_hit = hit_temp;
          } else if constexpr (std::is_same_v<T, uint32_t>) {
            prims[prim_index]->hit_check(ray);
            // increment whenever a hit test is done on a primitive
            ++return_variable;
          } else if constexpr (std::is_same_v<T, bool>) {
            const auto hit_surf_ptr = prims[prim_index]->hit_check(ray);
            // exit on first hit
            if (hit_surf_ptr) return true;
          }
        }
      } else {
        // intersect with both children and push further bbox first
        size_t first_child = node.first_index;
        size_t sec_child = first_child + 1;

        size_t bb_left_min = first_child * 2 + 2;
        size_t bb_left_max = bb_left_min + 2;

#ifdef __AVX2__
        auto [bb_hit1, bb_hit2]
            = ray_2aabb_slab(BB_mins_maxes[bb_left_min].data(), BB_mins_maxes[bb_left_max].data(),
                             ray_o, ray_dir_inv, ray);
#else
        size_t bb_right_min = bb_left_min + 1;
        size_t bb_right_max = bb_left_max + 1;

        float bb_hit1 = slab_intersect_aabb_array(ray, ray_inv_dir, BB_mins_maxes[bb_left_min],
                                                  BB_mins_maxes[bb_left_max]);
        float bb_hit2 = slab_intersect_aabb_array(ray, ray_inv_dir, BB_mins_maxes[bb_right_min],
                                                  BB_mins_maxes[bb_right_max]);
#endif

        if constexpr (std::is_same_v<T, bool>) {
          // don't care about order in hitcheck. as exit on first hit
          if (!std::isinf(bb_hit1)) thread_stack.push_back(first_child);
          if (!std::isinf(bb_hit2)) thread_stack.push_back(sec_child);
        } else {
          // order by distance
          if (!std::isinf(bb_hit2)) {
            if (!std::isinf(bb_hit1)) {
              // both intersect. swap according to dist. push more distant bbox first
              if (bb_hit2 > bb_hit1) {
                std::swap(first_child, sec_child);
              }
              thread_stack.push_back(first_child);
            }
            thread_stack.push_back(sec_child);
          } else if (!std::isinf(bb_hit1)) {
            thread_stack.push_back(first_child);
          }  // else don't push any child node
        }
      }
    }

    if constexpr (std::is_same_v<T, std::optional<HitInfo>>) {
      if (intermediate_hit.has_value()) {
        auto& pre_calc = intermediate_hit.value();
        return_variable = pre_calc.prim->hit_info(ray, pre_calc);
      }
    }

    return return_variable;
  }
};
