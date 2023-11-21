#include <bvh.h>

#include <algorithm>
#include <cstdint>
#include <numeric>

static size_t bin_index(uint8_t axis, const AABB& bbox, const glm::vec3& center,
                        const size_t num_bins) {
  int index = (center[axis] - bbox.box_min[axis])
              * (num_bins / (bbox.box_max[axis] - bbox.box_min[axis]));
  return std::min(num_bins - 1, static_cast<size_t>(std::max(0, index)));
}

// get best split on all axis
Split get_best_split(const BVHNode& node, const std::vector<size_t>& obj_indices,
                     const AABB* bboxes, const glm::vec3* centers, const size_t num_bins) {
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
                            const std::vector<AABB>& bboxes,
                            const std::vector<glm::vec3>& centers) {
  auto& curr_node = bvh.nodes[node_index];

  curr_node.aabb = bboxes[bvh.obj_indices[curr_node.first_index]];
  // set the AABB of current node
  for (size_t i = 1; i < curr_node.obj_count; ++i)
    curr_node.aabb.extend(bboxes[bvh.obj_indices[curr_node.first_index + i]]);
}

// Build and return BVH
BVH BVH::build(const std::vector<AABB>& bboxes, const std::vector<glm::vec3>& centers) {
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
  build_recursive(bvh, 0, node_count, bboxes, centers);

  bvh.nodes.resize(node_count);

  return bvh;
}
