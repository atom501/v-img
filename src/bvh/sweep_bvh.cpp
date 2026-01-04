#include <bvh.h>

#include <algorithm>
#include <thread>

static Split sweep_best_span_split(uint8_t axis, std::span<size_t> prim_sorted,
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

static void stable_partition(uint8_t fixed_axis, const std::vector<uint8_t>& is_left_mask,
                             std::array<std::span<size_t>, 4>& prim_axis_sort,
                             const size_t first_right) {
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
                      std::ref(left_depth), max_node_prims, ref(is_left_mask));
  } else {
    build_sweep_recursive(bvh, first_child, first_child * 2 + 2, node_count, bboxes,
                          left_prim_axis_sort, left_depth, max_node_prims, is_left_mask);
  }

  build_sweep_recursive(bvh, first_child + 1, first_child * 2 + 3, node_count, bboxes,
                        right_prim_axis_sort, right_depth, max_node_prims, is_left_mask);

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