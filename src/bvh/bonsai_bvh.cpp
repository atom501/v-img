#include <bvh.h>

#include <algorithm>
#include <mutex>
#include <numeric>
#include <thread>

// split into groups using midpoints. returns the indices of splits in the prim_indices array
static void midpoint_split(const std::vector<glm::vec3>& mid_points, std::span<size_t> prim_indices,
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

static size_t get_depth(const BVHNode& node, const std::vector<BVHNode>& list_of_nodes,
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

/*
static void combine_minitrees(BVH& BVH_of_mini_trees, const std::vector<BVH>& mini_trees,
                              const size_t total_obj_count) {
  size_t total_nodes = BVH_of_mini_trees.nodes.size();
  size_t obj_offset = 0;

  std::vector<size_t> obj_indices_offset;  // offset of obj_indices for each tree
  std::vector<size_t> node_offset;         // offset of nodes for each tree

  // calcaulate max depth for final BVH
  size_t max_depth = get_depth(BVH_of_mini_trees.nodes[0], BVH_of_mini_trees.nodes, mini_trees);

  obj_indices_offset.reserve(mini_trees.size());
  node_offset.reserve(mini_trees.size());
  for (const auto& tree : mini_trees) {
    obj_indices_offset.push_back(obj_offset);
    obj_offset += tree.obj_indices.size();

    node_offset.push_back(total_nodes - 1);
    total_nodes += tree.nodes.size() - 1;
  }

  assert(obj_offset == total_obj_count);

  // copy roots of mini tree into BVH_of_mini_trees
  for (auto& node : BVH_of_mini_trees.nodes) {
    if (!node.is_leaf()) continue;

    assert(node.obj_count == 1);
    // copy root node of mini tree
    const BVHNode& mini_tree_root = mini_trees[node.first_index].nodes[0];
    uint32_t tree_idx = node.first_index;

    size_t offset = mini_tree_root.is_leaf() ? obj_indices_offset[tree_idx] : node_offset[tree_idx];

    node = mini_tree_root;
    node.first_index = mini_tree_root.first_index + offset;
  }

  // combine mini_trees into BVH_of_mini_trees
  BVH_of_mini_trees.nodes.resize(total_nodes);
  BVH_of_mini_trees.BB_mins_maxes.resize(total_nodes * 2 + 3);
  BVH_of_mini_trees.obj_indices.resize(obj_offset);
  BVH_of_mini_trees.max_depth = max_depth;

#pragma omp parallel for
  for (size_t tree_idx = 0; tree_idx < mini_trees.size(); tree_idx++) {
    const auto& tree = mini_trees[tree_idx];

    // copy object indices
    std::copy(tree.obj_indices.begin(), tree.obj_indices.end(),
              BVH_of_mini_trees.obj_indices.begin() + obj_indices_offset[tree_idx]);

    // copy Nodes, skip root
    for (size_t node_idx = 1; node_idx < tree.nodes.size(); node_idx++) {
      auto& final_node = BVH_of_mini_trees.nodes[node_offset[tree_idx] + node_idx];
      const auto& tree_node = tree.nodes[node_idx];

      final_node = tree_node;

      size_t offset = tree_node.is_leaf() ? obj_indices_offset[tree_idx] : node_offset[tree_idx];

      final_node.first_index = tree_node.first_index + offset;
    }

    // copy AABBs, skip root AABB and padding
    std::copy(tree.BB_mins_maxes.begin() + 4, tree.BB_mins_maxes.end() - 1,
              BVH_of_mini_trees.BB_mins_maxes.begin() + ((node_offset[tree_idx] + 1) * 2 + 2));
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
    std::sort(split_indices.begin(), split_indices.end());

    std::span<size_t> span_obj_indices(obj_indices);
    std::vector<std::span<size_t>> spans_for_groups(split_indices.size() + 1);

    size_t left_start = 0;
    for (size_t i = 0; i < split_indices.size(); i++) {
      spans_for_groups[i] = span_obj_indices.subspan(left_start, split_indices[i] - left_start);
      left_start = split_indices[i];
    }
    spans_for_groups.back() = span_obj_indices.last(span_obj_indices.size() - left_start);

    std::vector<BVH> mini_trees(spans_for_groups.size());

// loop over groups and make a list of BVHs
#pragma omp parallel for
    for (size_t i = 0; i < mini_trees.size(); i++) {
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
*/