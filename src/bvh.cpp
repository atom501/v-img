#include <bvh.h>

#include <algorithm>
#include <cstdint>
#include <mutex>
#include <numeric>
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
static void build_recursive(BVH& bvh, size_t node_index, size_t bb_index,
                            std::atomic<size_t>& node_count, const std::vector<AABB>& bboxes,
                            const std::vector<glm::vec3>& centers, const size_t num_bins,
                            uint32_t& max_depth) {
  auto& curr_node = bvh.nodes[node_index];
  AABB curr_node_aabb = bboxes[bvh.obj_indices[curr_node.first_index]];
  // set the AABB of current node
  for (size_t i = 1; i < curr_node.obj_count; ++i)
    curr_node_aabb.extend(bboxes[bvh.obj_indices[curr_node.first_index + i]]);

  bvh.BB_mins_maxes[bb_index][0] = curr_node_aabb.bboxes[0].x;
  bvh.BB_mins_maxes[bb_index][1] = curr_node_aabb.bboxes[0].y;
  bvh.BB_mins_maxes[bb_index][2] = curr_node_aabb.bboxes[0].z;

  bvh.BB_mins_maxes[bb_index + 2][0] = curr_node_aabb.bboxes[1].x;
  bvh.BB_mins_maxes[bb_index + 2][1] = curr_node_aabb.bboxes[1].y;
  bvh.BB_mins_maxes[bb_index + 2][2] = curr_node_aabb.bboxes[1].z;

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
    } else {
      // Terminate with a leaf
      max_depth = 1;
      return;
    }
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
  std::thread left_thread;
  uint32_t left_depth = 0;
  uint32_t right_depth = 0;

  if (left.obj_count > 1024) {
    left_thread = std::thread(build_recursive, std::ref(bvh), first_child, first_child * 2 + 2,
                              std::ref(node_count), std::ref(bboxes), std::ref(centers), num_bins,
                              std::ref(left_depth));
  } else {
    build_recursive(bvh, first_child, first_child * 2 + 2, node_count, bboxes, centers, num_bins,
                    left_depth);
  }

  build_recursive(bvh, first_child + 1, first_child * 2 + 3, node_count, bboxes, centers, num_bins,
                  right_depth);

  if (left_thread.joinable()) {
    left_thread.join();
  }

  if (left_depth > right_depth) {
    max_depth = 1 + left_depth;
  } else {
    max_depth = 1 + right_depth;
  }
}

// Build and return BVH
BVH BVH::build_bin_bvh(const std::vector<AABB>& bboxes, const std::vector<glm::vec3>& centers,
                       const size_t num_bins) {
  BVH bvh;
  size_t obj_count = centers.size();

  if (obj_count > 0) {
    // initialise indices pointing to original object list
    bvh.obj_indices.resize(obj_count);
    std::iota(bvh.obj_indices.begin(), bvh.obj_indices.end(), 0);

    // resize to maximum nodes possible
    bvh.nodes.resize(2 * obj_count - 1);
    bvh.BB_mins_maxes.resize((2 * obj_count - 1) * 2 + 3);

    bvh.nodes[0].obj_count = obj_count;
    bvh.nodes[0].first_index = 0;

    std::atomic<size_t> node_count = 1;
    uint32_t max_depth = 0;

    // build the bvh top down
    build_recursive(bvh, 0, 0, node_count, bboxes, centers, num_bins, max_depth);

    bvh.nodes.resize(node_count);

    // added padding at the end as simd loads 8 values
    bvh.BB_mins_maxes.resize(node_count * 2 + 3);

    bvh.max_depth = max_depth;
  }

  return bvh;
}

// split into groups using midpoints. returns the indices of splits in the prim_indices array
void midpoint_split(const std::vector<glm::vec3>& mid_points, std::span<size_t> prim_indices,
                    const size_t* prim_begin, std::vector<size_t>& split_indices,
                    std::mutex& index_mutex) {
  // exit if number of objects less than threshold. no splitting then
  if (prim_indices.size() <= 512) {
    return;
  }

  auto& m0 = mid_points[prim_indices[0]];
  auto& m1 = mid_points[prim_indices[1]];

  AABB group_bb = AABB(glm::min(m0, m1), glm::max(m0, m1));

  for (size_t i = 2; i < prim_indices.size(); i++) {
    group_bb.extend(mid_points[prim_indices[i]]);
  }

  int8_t largest_axis = group_bb.largest_axis();

  float mid_of_axis = (group_bb.bboxes[1][largest_axis] + group_bb.bboxes[0][largest_axis]) / 2.f;

  size_t first_right_span
      = std::partition(prim_indices.begin(), prim_indices.end(),
                       [&](size_t i) { return mid_points[i][largest_axis] < mid_of_axis; })
        - prim_indices.begin();

  // get where split is in the original prim_indices array and save index
  size_t first_right = prim_indices.data() - prim_begin + first_right_span;

  index_mutex.lock();
  split_indices.push_back(first_right);
  index_mutex.unlock();

  // recursively split left and right partition. make new thread for left if greater than 1024 size
  auto left_span = prim_indices.first(first_right_span);
  auto right_span = prim_indices.last(prim_indices.size() - first_right_span);

  std::jthread left_thread;
  if (left_span.size() > 1024) {
    left_thread = std::jthread(midpoint_split, std::ref(mid_points), left_span, prim_begin,
                               std::ref(split_indices), std::ref(index_mutex));
  } else {
    midpoint_split(mid_points, left_span, prim_begin, split_indices, index_mutex);
  }

  midpoint_split(mid_points, right_span, prim_begin, split_indices, index_mutex);
}

size_t get_depth(const BVHNode& node, const std::vector<BVHNode>& list_of_nodes,
                 const std::vector<BVH>& mini_trees) {
  if (node.is_leaf()) {
    assert(node.obj_count == 1);
    return mini_trees[node.first_index].max_depth;
  } else {
    const auto& left_node = list_of_nodes[node.first_index];
    const auto& right_node = list_of_nodes[node.first_index + 1];

    size_t left_depth = 1 + get_depth(left_node, list_of_nodes, mini_trees);
    size_t right_depth = 1 + get_depth(right_node, list_of_nodes, mini_trees);

    if (left_depth > right_depth)
      return left_depth;
    else
      return right_depth;
  }
}

void combine_minitrees(BVH& BVH_of_mini_trees, const std::vector<BVH>& mini_trees,
                       const size_t total_obj_count) {
  size_t total_nodes = BVH_of_mini_trees.nodes.size();
  size_t copy_begin = 0;

  std::vector<size_t> obj_indices_offset;  // offset of obj_indices for each tree
  std::vector<size_t> node_offset;         // offset of nodes for each tree

  // calcaulate max depth for final BVH
  size_t max_depth = get_depth(BVH_of_mini_trees.nodes[0], BVH_of_mini_trees.nodes, mini_trees);

  obj_indices_offset.reserve(mini_trees.size());
  node_offset.reserve(mini_trees.size());
  for (const auto& tree : mini_trees) {
    obj_indices_offset.push_back(copy_begin);
    copy_begin += tree.obj_indices.size();

    node_offset.push_back(total_nodes - 1);
    total_nodes += tree.nodes.size() - 1;
  }

  // copy roots of mini tree into BVH_of_mini_trees
  for (auto& node : BVH_of_mini_trees.nodes) {
    if (!node.is_leaf()) continue;

    assert(node.obj_count == 1);
    // copy root node of mini tree
    const BVHNode& mini_tree_root = mini_trees[node.first_index].nodes[0];

    node = mini_tree_root;
    size_t offset = mini_tree_root.is_leaf() ? obj_indices_offset[node.first_index]
                                             : node_offset[node.first_index];

    node.first_index = mini_tree_root.first_index + offset;
  }

  // combine mini_trees into BVH_of_mini_trees
  BVH_of_mini_trees.nodes.resize(total_nodes);
  BVH_of_mini_trees.BB_mins_maxes.resize(total_nodes * 2 + 3);
  BVH_of_mini_trees.obj_indices.resize(total_obj_count);
  BVH_of_mini_trees.max_depth = max_depth;

#pragma omp parallel for
  for (size_t tree_idx = 0; tree_idx < mini_trees.size(); tree_idx++) {
    const auto& tree = mini_trees[tree_idx];

    // copy object indices
    std::copy(tree.obj_indices.begin(), tree.obj_indices.end(),
              BVH_of_mini_trees.obj_indices.begin() + obj_indices_offset[tree_idx]);

    // copy Nodes, skip root
    for (size_t node_idx = 1; node_idx < tree.nodes.size(); node_idx++) {
      BVH_of_mini_trees.nodes[node_offset[tree_idx] + node_idx] = tree.nodes[node_idx];

      size_t offset
          = tree.nodes[node_idx].is_leaf() ? obj_indices_offset[tree_idx] : node_offset[tree_idx];

      BVH_of_mini_trees.nodes[node_offset[tree_idx] + node_idx].first_index = offset;
    }

    // copy AABBs, skip root AABB and padding
    std::copy(tree.BB_mins_maxes.begin() + 4, tree.BB_mins_maxes.end() - 1,
              BVH_of_mini_trees.BB_mins_maxes.begin() + (node_offset[tree_idx] * 2 + 2));
  }
}

BVH BVH::build_bonsai_bvh(const std::vector<AABB>& bboxes, bool prune,
                          const uint16_t max_node_primes) {
  BVH bvh;
  size_t obj_count = bboxes.size();

  if (obj_count > 0) {
    // create mid points of bounding boxes
    std::vector<glm::vec3> bb_mid_points(obj_count);

#pragma omp parallel for
    for (size_t i = 0; i < bb_mid_points.size(); i++) {
      bb_mid_points[i] = (bboxes[i].bboxes[0] + bboxes[i].bboxes[1]) / 2.f;
    }

    std::vector<size_t> obj_indices(obj_count);
    std::iota(obj_indices.begin(), obj_indices.end(), 0);

    std::vector<size_t> split_indices;
    std::mutex mutex;

    // obj_indices is split into groups. split_indices have index of first right of a split
    midpoint_split(bb_mid_points, obj_indices, obj_indices.data(), split_indices, mutex);

    std::span<size_t> span_obj_indices(obj_indices);
    std::vector<std::span<size_t>> spans_for_groups(split_indices.size() + 1);

    size_t left_start = 0;
    for (size_t i = 0; i < split_indices.size(); i++) {
      spans_for_groups[i] = span_obj_indices.subspan(left_start, split_indices[i]);
      left_start = split_indices[i];
    }
    spans_for_groups.back() = span_obj_indices.last(span_obj_indices.size() - left_start);

    std::vector<BVH> mini_trees(spans_for_groups.size());

// loop over groups and make a list of BVHs
#pragma omp parallel for
    for (size_t i = 0; i < spans_for_groups.size(); i++) {
      mini_trees[i] = build_sweep_bvh(bboxes, bb_mid_points, spans_for_groups[i], max_node_primes);
    }

    // Bonsai pruning

    // Build tree from mini trees

    // get AABB of root node for each BVH
    std::vector<AABB> bvh_AABBs(mini_trees.size());

    for (size_t i = 0; i < bvh_AABBs.size(); i++) {
      bvh_AABBs[i]
          = AABB(glm::vec3(mini_trees[i].BB_mins_maxes[0][0], mini_trees[i].BB_mins_maxes[0][1],
                           mini_trees[i].BB_mins_maxes[0][2]),
                 glm::vec3(mini_trees[i].BB_mins_maxes[2][0], mini_trees[i].BB_mins_maxes[2][1],
                           mini_trees[i].BB_mins_maxes[2][2]));
    }

    // center for each BVH root node AABB
    std::vector<glm::vec3> bvh_bb_mid_points(mini_trees.size());

#pragma omp parallel for
    for (size_t i = 0; i < bvh_bb_mid_points.size(); i++) {
      bvh_bb_mid_points[i] = (bvh_AABBs[i].bboxes[0] + bvh_AABBs[i].bboxes[1]) / 2.f;
    }

    std::vector<size_t> bvh_indices(mini_trees.size());
    std::iota(bvh_indices.begin(), bvh_indices.end(), 0);

    // max node primitives set to 1 as all nodes can only have 1 mini tree
    BVH BVH_of_mini_trees = build_sweep_bvh(bvh_AABBs, bvh_bb_mid_points, bvh_indices, 1);

    // Use the indices inside BVH_of_mini_trees to combine the trees
    combine_minitrees(BVH_of_mini_trees, mini_trees, obj_count);

    bvh = BVH_of_mini_trees;
  }

  return bvh;
}

Split sweep_best_span_split(uint8_t axis, std::span<size_t> prim_sorted,
                            const std::vector<AABB>& bboxes, const Split& prev_split) {
  size_t first_left = 0;
  size_t num_prims = prim_sorted.size();
  Bin extend_from_right(0, AABB(glm::vec3(+std::numeric_limits<float>::max()),
                                glm::vec3(-std::numeric_limits<float>::max())));
  Split best_split = prev_split;

  std::vector<float> right_costs(num_prims, std::numeric_limits<float>::infinity());

  for (int64_t i = num_prims - 1; i > 0; i--) {
    extend_from_right.aabb.extend(bboxes[prim_sorted[i]]);
    extend_from_right.obj_count++;

    right_costs[i] = extend_from_right.cost();

    if (right_costs[i] > best_split.cost) {
      first_left = i - 1;
      break;
    }
  }

  Bin extend_from_left(0, AABB(glm::vec3(+std::numeric_limits<float>::max()),
                               glm::vec3(-std::numeric_limits<float>::max())));

  for (size_t i = 0; i < first_left; i++) {
    extend_from_left.aabb.extend(bboxes[prim_sorted[i]]);
    extend_from_left.obj_count++;
  }

  for (size_t i = first_left; i < num_prims - 1; i++) {
    extend_from_left.aabb.extend(bboxes[prim_sorted[i]]);
    extend_from_left.obj_count++;

    float left_cost = extend_from_left.cost();
    float total_cost = left_cost + right_costs[i + 1];

    if (total_cost < best_split.cost) best_split = Split{i + 1, total_cost, axis};
  }

  return best_split;
}

void stable_partition(uint8_t fixed_axis, const std::vector<uint8_t>& is_left_mask,
                      std::array<std::span<size_t>, 4>& prim_axis_sort, const size_t first_right) {
  for (size_t axis = 0; axis < 3; axis++) {
    if (axis == fixed_axis) continue;

    size_t left_count = 0;
    size_t right_count = 0;

    for (size_t i = 0; i < prim_axis_sort[axis].size(); i++) {
      if (is_left_mask[prim_axis_sort[axis][i]]) {
        prim_axis_sort[3][left_count] = prim_axis_sort[axis][i];
        left_count++;
      } else {
        prim_axis_sort[3][first_right + right_count] = prim_axis_sort[axis][i];
        right_count++;
      }
    }

    std::swap(prim_axis_sort[3], prim_axis_sort[axis]);
  }
}

static void build_sweep_recursive(BVH& bvh, size_t node_index, size_t bb_index,
                                  std::atomic<size_t>& node_count, const std::vector<AABB>& bboxes,
                                  std::array<std::span<size_t>, 4>& prim_axis_sort,
                                  uint32_t& max_depth, const uint32_t max_node_prims,
                                  std::vector<uint8_t>& is_left_mask) {
  auto& curr_node = bvh.nodes[node_index];
  size_t total_objs = bboxes.size();
  size_t curr_node_objs = prim_axis_sort[0].size();

  // all sorted axis contain the same primitives. use one to create AABB over all primitives
  AABB curr_node_aabb = bboxes[prim_axis_sort[0][0]];
  for (size_t i = 1; i < curr_node_objs; i++) {
    curr_node_aabb.extend(bboxes[prim_axis_sort[0][i]]);
  }

  // set AABB for current node
  for (size_t i = 0; i < 3; i++) {
    bvh.BB_mins_maxes[bb_index][i] = curr_node_aabb.bboxes[0][i];
    bvh.BB_mins_maxes[bb_index + 2][i] = curr_node_aabb.bboxes[1][i];
  }

  if (curr_node_objs == 1) {
    max_depth = 1;
    bvh.obj_indices[curr_node.first_index] = prim_axis_sort[0][0];
    return;
  }

  const float leaf_cost = curr_node_aabb.half_SA() * (curr_node_objs - 1);
  // get split for all axis
  Split best_split = Split{prim_axis_sort[0].size() / 2, leaf_cost, 0};

  for (size_t i = 0; i < 3; i++) {
    best_split = sweep_best_span_split(i, prim_axis_sort[i], bboxes, best_split);
  }

  // use median split if many primitives but cost is high. else use best_split
  if (best_split.cost >= leaf_cost) {
    if (curr_node_objs > max_node_prims) {
      // use the median split if cost too high but more than max_node_prims
      best_split.axis = curr_node_aabb.largest_axis();
      best_split.bin_split = prim_axis_sort[0].size() / 2;
    } else {
      // else set as leaf and write final prims to obj_indices
      max_depth = 1;

      std::copy(prim_axis_sort[0].begin(), prim_axis_sort[0].end(),
                bvh.obj_indices.begin() + curr_node.first_index);

      return;
    }
  }

  // split each axis of prim_axis_sort. preserving order
  for (size_t i = best_split.bin_split; i < prim_axis_sort[best_split.axis].size(); i++) {
    is_left_mask[prim_axis_sort[best_split.axis][i]] = 0;
  }
  for (size_t i = 0; i < best_split.bin_split; i++) {
    is_left_mask[prim_axis_sort[best_split.axis][i]] = 1;
  }

  stable_partition(best_split.axis, is_left_mask, prim_axis_sort, best_split.bin_split);

  std::array<std::span<size_t>, 4> left_prim_axis_sort = {
      prim_axis_sort[0].first(best_split.bin_split), prim_axis_sort[1].first(best_split.bin_split),
      prim_axis_sort[2].first(best_split.bin_split), prim_axis_sort[3].first(best_split.bin_split)};

  std::array<std::span<size_t>, 4> right_prim_axis_sort
      = {prim_axis_sort[0].last(prim_axis_sort[0].size() - best_split.bin_split),
         prim_axis_sort[1].last(prim_axis_sort[1].size() - best_split.bin_split),
         prim_axis_sort[2].last(prim_axis_sort[2].size() - best_split.bin_split),
         prim_axis_sort[3].last(prim_axis_sort[2].size() - best_split.bin_split)};

  // set children and recurse. set make thread for left if greater than 1024
  size_t first_child = node_count.fetch_add(2);
  auto& left = bvh.nodes[first_child];
  auto& right = bvh.nodes[first_child + 1];

  left.obj_count = left_prim_axis_sort[0].size();
  right.obj_count = right_prim_axis_sort[0].size();

  // points to index in obj_indices
  left.first_index = curr_node.first_index;
  right.first_index = left.first_index + left.obj_count;

  curr_node.first_index = first_child;
  curr_node.obj_count = 0;

  // run left in another thread if greater than 1024 primitives
  std::thread left_thread;
  uint32_t left_depth = 0;
  uint32_t right_depth = 0;

  if (left.obj_count >= 1024) {
    left_thread
        = std::thread(build_sweep_recursive, std::ref(bvh), first_child, first_child * 2 + 2,
                      std::ref(node_count), std::ref(bboxes), std::ref(left_prim_axis_sort),
                      std::ref(left_depth), 8, ref(is_left_mask));
  } else {
    build_sweep_recursive(bvh, first_child, first_child * 2 + 2, node_count, bboxes,
                          left_prim_axis_sort, left_depth, 8, is_left_mask);
  }

  build_sweep_recursive(bvh, first_child + 1, first_child * 2 + 3, node_count, bboxes,
                        right_prim_axis_sort, right_depth, 8, is_left_mask);

  if (left_thread.joinable()) {
    left_thread.join();
  }

  if (left_depth > right_depth) {
    max_depth = 1 + left_depth;
  } else {
    max_depth = 1 + right_depth;
  }
}

BVH BVH::build_sweep_bvh(const std::vector<AABB>& bboxes, const std::vector<glm::vec3>& centers,
                         std::span<size_t> prim_indices, const uint16_t max_node_primes) {
  BVH bvh;
  size_t obj_count = prim_indices.size();

  if (obj_count > 0) {
    bvh.obj_indices.assign(prim_indices.begin(), prim_indices.end());

    // sort primitve along each axis. order will be preserved during recursion.
    // 4th is used as temp storage for partition
    std::array<std::vector<size_t>, 4> prim_axis_sort;
    for (size_t axis = 0; axis < 3; axis++) {
      prim_axis_sort[axis].assign(prim_indices.begin(), prim_indices.end());
      std::sort(prim_axis_sort[axis].begin(), prim_axis_sort[axis].end(),
                [&](size_t i, size_t j) { return centers[i][axis] < centers[j][axis]; });
    }
    prim_axis_sort[3].assign(prim_indices.begin(), prim_indices.end());

    // resize to maximum nodes possible
    bvh.nodes.resize(2 * obj_count - 1);
    bvh.BB_mins_maxes.resize((2 * obj_count - 1) * 2 + 3);

    bvh.nodes[0].obj_count = obj_count;
    bvh.nodes[0].first_index = 0;

    std::atomic<size_t> node_count = 1;
    uint32_t max_depth = 0;

    std::array<std::span<size_t>, 4> sorted_span
        = {std::span(prim_axis_sort[0]), std::span(prim_axis_sort[1]), std::span(prim_axis_sort[2]),
           std::span(prim_axis_sort[3])};

    // std::vector<uint8_t> used instead of std::vector<bool> does not need locks for shared writes
    std::vector<uint8_t> is_left_mask(bboxes.size(), 0);

    // build sweep bvh recursively
    build_sweep_recursive(bvh, 0, 0, node_count, bboxes, sorted_span, max_depth, max_node_primes,
                          is_left_mask);

    bvh.nodes.resize(node_count);

    // added padding at the end as simd loads 8 values
    bvh.BB_mins_maxes.resize(node_count * 2 + 3);

    bvh.max_depth = max_depth;
  }

  return bvh;
}
