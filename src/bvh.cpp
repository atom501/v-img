#include <bvh.h>

#include <algorithm>
#include <cstdint>
#include <numeric>
#include <stack>

static size_t bin_index(uint8_t axis, const AABB& bbox, const glm::vec3& center,
                        const size_t num_bins) {
  int index = (center[axis] - bbox.bboxes[0][axis])
              * (num_bins / (bbox.bboxes[1][axis] - bbox.bboxes[0][axis]));
  return std::min(num_bins - 1, static_cast<size_t>(std::max(0, index)));
}

// get best split on all axis
Split get_best_split(const BVHNode& node, const std::vector<size_t>& obj_indices,
                     const std::vector<AABB>& bboxes, const std::vector<glm::vec3>& centers,
                     const size_t num_bins) {
  Split best_split = {std::numeric_limits<size_t>::max(), std::numeric_limits<float>::max(),
                      std::numeric_limits<uint8_t>::max()};

  // temp bin for initialisation
  Bin init_bin(0, AABB(glm::vec3(+std::numeric_limits<float>::max()),
                       glm::vec3(-std::numeric_limits<float>::max())));

  // check splitting for all axis
  for (size_t axis = 0; axis < 3; axis++) {
    std::vector<Bin> bins(num_bins, init_bin);

    // filling bins
    for (size_t i = 0; i < node.obj_count; i++) {
      auto obj_index = obj_indices[node.first_index + i];
      auto& bin = bins[bin_index(axis, node.aabb, centers[obj_index], num_bins)];
      bin.aabb.extend(bboxes[obj_index]);
      bin.obj_count++;
    }

    // save costs for right side
    std::vector<float> right_cost(num_bins, 0.0f);
    Bin extend_from_right(0, AABB(glm::vec3(+std::numeric_limits<float>::max()),
                                  glm::vec3(-std::numeric_limits<float>::max())));

    // start extending from the right to left. saving costs in an array
    for (size_t i = num_bins - 1; i > 0; --i) {
      extend_from_right.aabb.extend(bins[i].aabb);
      extend_from_right.obj_count += bins[i].obj_count;
      // cost of an empty bin is -NaN
      right_cost[i] = extend_from_right.cost();
    }

    Bin extend_from_left(0, AABB(glm::vec3(+std::numeric_limits<float>::max()),
                                 glm::vec3(-std::numeric_limits<float>::max())));

    for (size_t i = 0; i < num_bins - 1; ++i) {
      extend_from_left.aabb.extend(bins[i].aabb);
      extend_from_left.obj_count += bins[i].obj_count;
      float cost = extend_from_left.cost() + right_cost[i + 1];
      /*
       * This test is defined such that NaNs are automatically ignored.
       * Thus, only valid combinations with non-empty bins are considered.
       * IEEE-754 standard guarantees that a comparison which involves a NaN value evaluates to
       * false
       */
      if (cost < best_split.cost) {
        best_split.cost = cost;
        best_split.bin_split = i + 1;
        best_split.axis = axis;
      }
    }
  }

  return best_split;
}

// helper bvh constructor
static void build_recursive(BVH& bvh, size_t node_index, size_t& node_count,
                            const std::vector<AABB>& bboxes, const std::vector<glm::vec3>& centers,
                            const size_t num_bins) {
  auto& curr_node = bvh.nodes[node_index];

  curr_node.aabb = bboxes[bvh.obj_indices[curr_node.first_index]];
  // set the AABB of current node
  for (size_t i = 1; i < curr_node.obj_count; ++i)
    curr_node.aabb.extend(bboxes[bvh.obj_indices[curr_node.first_index + i]]);

  Split best_split = get_best_split(curr_node, bvh.obj_indices, bboxes, centers, num_bins);

  // use the best_split

  float curr_leaf_cost
      = curr_node.aabb.half_SA() * (curr_node.obj_count - 1);  // TODO change 1 to be configurable

  size_t first_right;  // Index of the first primitive in the right child

  // condition for not splitting
  if ((best_split.bin_split == std::numeric_limits<float>::max())
      || best_split.cost >= curr_leaf_cost) {
    if (curr_node.obj_count > 8) {  // TODO change max prim to be configurable
      // use the median split if many primitives
      uint8_t axis = curr_node.aabb.largest_axis();

      std::sort(bvh.obj_indices.begin() + curr_node.first_index,
                bvh.obj_indices.begin() + curr_node.first_index + curr_node.obj_count,
                [&](size_t i, size_t j) { return centers[i][axis] < centers[j][axis]; });

      first_right = curr_node.first_index + curr_node.obj_count / 2;
    } else
      // Terminate with a leaf
      return;
  } else {
    /* The split was good, we need to partition the primitives. subtracted with
     * bvh.obj_indices.begin() to turn it into index and not get an iterator
     */
    first_right
        = std::partition(bvh.obj_indices.begin() + curr_node.first_index,
                         bvh.obj_indices.begin() + curr_node.first_index + curr_node.obj_count,
                         [&](size_t i) {
                           return bin_index(best_split.axis, curr_node.aabb, centers[i], num_bins)
                                  < best_split.bin_split;
                         })
          - bvh.obj_indices.begin();
  }

  size_t first_child = node_count;
  auto& left = bvh.nodes[first_child];
  auto& right = bvh.nodes[first_child + 1];
  node_count += 2;

  left.obj_count = first_right - curr_node.first_index;
  right.obj_count = curr_node.obj_count - left.obj_count;

  left.first_index = curr_node.first_index;
  right.first_index = first_right;

  curr_node.first_index = first_child;
  curr_node.obj_count = 0;

  build_recursive(bvh, first_child, node_count, bboxes, centers, num_bins);
  build_recursive(bvh, first_child + 1, node_count, bboxes, centers, num_bins);
}

// Build and return BVH
BVH BVH::build(const std::vector<AABB>& bboxes, const std::vector<glm::vec3>& centers,
               const size_t num_bins) {
  BVH bvh;
  size_t obj_count = centers.size();

  // initialise indices pointing to original object list
  bvh.obj_indices.resize(obj_count);
  std::iota(bvh.obj_indices.begin(), bvh.obj_indices.end(), 0);

  // resize to maximum nodes possible
  bvh.nodes.resize(2 * obj_count - 1);
  bvh.nodes[0].obj_count = obj_count;
  bvh.nodes[0].first_index = 0;

  size_t node_count = 1;
  // build the bvh top down
  build_recursive(bvh, 0, node_count, bboxes, centers, num_bins);

  bvh.nodes.resize(node_count);

  return bvh;
}

std::optional<HitInfo> BVH::hit(Ray& ray, std::vector<size_t>& thread_stack,
                                const std::vector<std::unique_ptr<Surface>>& prims) const {
  std::optional<HitInfo> hit_final = std::nullopt;
  std::optional<HitInfo> hit_temp = std::nullopt;
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

  if (!root_node.aabb.intersect(ray, ray_inv_dir)) return std::nullopt;

  thread_stack.push_back(0);

  while (!thread_stack.empty()) {
    auto& node = nodes[thread_stack.back()];
    thread_stack.pop_back();

    // if leaf check all primitives in the leaf
    if (node.is_leaf()) {
      for (size_t i = 0; i < node.obj_count; ++i) {
        auto prim_index = obj_indices[node.first_index + i];

        // ray's tmax value will be changed if hit
        hit_temp = prims[prim_index]->hit(ray);
        // if ray hit the object replace the last hit_final
        if (hit_temp.has_value()) hit_final = hit_temp;
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

  return hit_final;
}

uint32_t BVH::hit_heatmap(Ray& ray, std::vector<size_t>& thread_stack,
                          const std::vector<std::unique_ptr<Surface>>& prims) const {
  std::optional<float> bb_hit1;
  std::optional<float> bb_hit2;

  /*
   * check root node first. Since children are always hit checked before being pushed onto the
   * stack, no need to perform hit test again when popping the node (except root node)
   */
  auto& root_node = nodes[0];

  uint32_t ray_prim_hit_count = 0;

  glm::vec3 ray_inv_dir;
  ray_inv_dir[0] = 1.0f / ray.dir[0];
  ray_inv_dir[1] = 1.0f / ray.dir[1];
  ray_inv_dir[2] = 1.0f / ray.dir[2];

  if (!root_node.aabb.intersect(ray, ray_inv_dir)) return ray_prim_hit_count;

  thread_stack.push_back(0);

  while (!thread_stack.empty()) {
    auto& node = nodes[thread_stack.back()];
    thread_stack.pop_back();

    // if leaf check all primitives in the leaf
    if (node.is_leaf()) {
      for (size_t i = 0; i < node.obj_count; ++i) {
        auto prim_index = obj_indices[node.first_index + i];

        // ray's tmax value will be changed if hit
        prims[prim_index]->hit(ray);
        // increment whenever a hit test is done on a primitive
        ++ray_prim_hit_count;
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

  return ray_prim_hit_count;
}
