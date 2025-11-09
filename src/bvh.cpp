#include <bvh.h>

#include <algorithm>
#include <cstdint>
#include <numeric>
#include <stack>
#include <thread>

static size_t bin_index(uint8_t axis, const AABB& bbox, const glm::vec3& center,
                        const size_t num_bins) {
  int index = (center[axis] - bbox.bboxes[0][axis])
              * (num_bins / (bbox.bboxes[1][axis] - bbox.bboxes[0][axis]));
  return std::min(num_bins - 1, static_cast<size_t>(std::max(0, index)));
}

// get best split on all axis
Split get_best_split(const BVHNode& node, const std::vector<size_t>& obj_indices,
                     const std::vector<AABB>& bboxes, const std::vector<glm::vec3>& centers,
                     const size_t num_bins, const AABB& aabb) {
  Split best_split = {std::numeric_limits<size_t>::max(), std::numeric_limits<float>::max(), 0};

  // temp bin for initialisation
  Bin init_bin(0, AABB(glm::vec3(+std::numeric_limits<float>::max()),
                       glm::vec3(-std::numeric_limits<float>::max())));

  // check splitting for all axis
  for (size_t axis = 0; axis < 3; axis++) {
    std::vector<Bin> bins(num_bins, init_bin);

    // filling bins
    for (size_t i = 0; i < node.obj_count; i++) {
      auto obj_index = obj_indices[node.first_index + i];
      auto& bin = bins[bin_index(axis, aabb, centers[obj_index], num_bins)];
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
static void build_recursive(BVH& bvh, size_t node_index, std::atomic<size_t>& node_count,
                            const std::vector<AABB>& bboxes, const std::vector<glm::vec3>& centers,
                            const size_t num_bins) {
  auto& curr_node = bvh.nodes[node_index];
  AABB curr_node_aabb = bboxes[bvh.obj_indices[curr_node.first_index]];
  // set the AABB of current node
  for (size_t i = 1; i < curr_node.obj_count; ++i)
    curr_node_aabb.extend(bboxes[bvh.obj_indices[curr_node.first_index + i]]);

  bvh.BB_mins[node_index][0] = curr_node_aabb.bboxes[0].x;
  bvh.BB_mins[node_index][1] = curr_node_aabb.bboxes[0].y;
  bvh.BB_mins[node_index][2] = curr_node_aabb.bboxes[0].z;

  bvh.BB_maxes[node_index][0] = curr_node_aabb.bboxes[1].x;
  bvh.BB_maxes[node_index][1] = curr_node_aabb.bboxes[1].y;
  bvh.BB_maxes[node_index][2] = curr_node_aabb.bboxes[1].z;

  Split best_split
      = get_best_split(curr_node, bvh.obj_indices, bboxes, centers, num_bins, curr_node_aabb);

  // use the best_split

  float curr_leaf_cost
      = curr_node_aabb.half_SA() * (curr_node.obj_count - 1);  // TODO change 1 to be configurable

  size_t first_right;  // Index of the first primitive in the right child

  // condition for not splitting
  if ((best_split.bin_split == std::numeric_limits<float>::max())
      || best_split.cost >= curr_leaf_cost) {
    if (curr_node.obj_count > 8) {  // TODO change max prim to be configurable
      // use the median split if many primitives
      uint8_t axis = curr_node_aabb.largest_axis();

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
                           return bin_index(best_split.axis, curr_node_aabb, centers[i], num_bins)
                                  < best_split.bin_split;
                         })
          - bvh.obj_indices.begin();
  }

  size_t first_child = node_count.fetch_add(2);
  auto& left = bvh.nodes[first_child];
  auto& right = bvh.nodes[first_child + 1];

  left.obj_count = first_right - curr_node.first_index;
  right.obj_count = curr_node.obj_count - left.obj_count;

  left.first_index = curr_node.first_index;
  right.first_index = first_right;

  curr_node.first_index = first_child;
  curr_node.obj_count = 0;

  // run left in another thread if greater than 1024 primitives
  std::jthread left_thread;
  if (left.obj_count > 1024) {
    left_thread = std::jthread(build_recursive, std::ref(bvh), first_child, std::ref(node_count),
                               std::ref(bboxes), std::ref(centers), num_bins);
  } else {
    build_recursive(bvh, first_child, node_count, bboxes, centers, num_bins);
  }

  build_recursive(bvh, first_child + 1, node_count, bboxes, centers, num_bins);
}

// Build and return BVH
BVH BVH::build(const std::vector<AABB>& bboxes, const std::vector<glm::vec3>& centers,
               const size_t num_bins) {
  BVH bvh;
  size_t obj_count = centers.size();

  if (obj_count > 0) {
    // initialise indices pointing to original object list
    bvh.obj_indices.resize(obj_count);
    std::iota(bvh.obj_indices.begin(), bvh.obj_indices.end(), 0);

    // resize to maximum nodes possible
    bvh.nodes.resize(2 * obj_count - 1);
    bvh.BB_mins.resize(2 * obj_count - 1);
    bvh.BB_maxes.resize(2 * obj_count - 1);

    bvh.nodes[0].obj_count = obj_count;
    bvh.nodes[0].first_index = 0;

    std::atomic<size_t> node_count = 1;
    // build the bvh top down
    build_recursive(bvh, 0, node_count, bboxes, centers, num_bins);

    bvh.nodes.resize(node_count);
    bvh.BB_mins.resize(node_count);
    bvh.BB_maxes.resize(node_count);
  }

  return bvh;
}
