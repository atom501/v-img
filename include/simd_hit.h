#pragma once

#ifdef __AVX2__
#  include <hit_utils.h>
#  include <immintrin.h>

// input (x, y, z, y). returns max(x, y, z, t)
inline float horizontal_max_128(__m128 x) {
  // shuffle to (z, t, x, x)
  __m128 shuffle1 = _mm_shuffle_ps(x, x, _MM_SHUFFLE(0, 0, 3, 2));

  // result (max(x,z), max(y, t), max(x, z), max(x,t))
  __m128 max1 = _mm_max_ps(x, shuffle1);

  // if max1 is ( m0, m1, m2, m3 ). shuffle2 is (m1, m0, m0, m0)
  __m128 shuffle2 = _mm_shuffle_ps(max1, max1, _MM_SHUFFLE(0, 0, 0, 1));

  // least significant element will be max(m0, m1). i.e max(max(x,z), max(y, t))
  __m128 max2 = _mm_max_ps(max1, shuffle2);

  // return least significant element
  return _mm_cvtss_f32(max2);
}

// input (x, y, z, y). returns min(x, y, z, t)
// same logic as horizontal_max_128 just using min operations
inline float horizontal_min_128(const __m128& x) {
  __m128 shuffle1 = _mm_shuffle_ps(x, x, _MM_SHUFFLE(0, 0, 3, 2));
  __m128 min1 = _mm_min_ps(x, shuffle1);
  __m128 shuffle2 = _mm_shuffle_ps(min1, min1, _MM_SHUFFLE(0, 0, 0, 1));
  __m128 min2 = _mm_min_ps(min1, shuffle2);
  return _mm_cvtss_f32(min2);
}

// aabb mins and maxes are given array of floats. Result is given in t1
// used slab_intersect_aabb_array function as basis for simd
inline float ray_1aabb_slab(const float* mins, const float* maxs, const __m256& ray_o,
                            const __m256& ray_dir_inv, const Ray& r) {
  // (bboxes_mins - ray.origin) * invRayDir
  __m128 bbox_min_vals = _mm_loadu_ps(mins);
  bbox_min_vals = _mm_mul_ps(_mm_sub_ps(bbox_min_vals, _mm256_castps256_ps128(ray_o)),
                             _mm256_castps256_ps128(ray_dir_inv));

  // (bboxes_maxes - ray.origin) * invRayDir
  __m128 bbox_max_vals = _mm_loadu_ps(maxs);
  bbox_max_vals = _mm_mul_ps(_mm_sub_ps(bbox_max_vals, _mm256_castps256_ps128(ray_o)),
                             _mm256_castps256_ps128(ray_dir_inv));

  // get element wise min and max. equivalent of tMins and tMaxes in slab_intersect_aabb_array
  __m128 min_elements = _mm_min_ps(bbox_min_vals, bbox_max_vals);
  __m128 max_elements = _mm_max_ps(bbox_min_vals, bbox_max_vals);

  // overwrite upper most value with r.minT
  __m128 ray_limit_val = _mm_set_ss(r.minT);
  __m128 elements_with_ray_limit = _mm_insert_ps(min_elements, ray_limit_val, 0b00'11'00'00);

  float tBoxMin = horizontal_max_128(elements_with_ray_limit);

  // overwrite upper most value with r.maxT
  ray_limit_val = _mm_set_ss(r.maxT);
  elements_with_ray_limit = _mm_insert_ps(max_elements, ray_limit_val, 0b00'11'00'00);
  float tBoxMax = horizontal_min_128(elements_with_ray_limit);

  if (tBoxMin <= tBoxMax)
    return tBoxMin;
  else
    return std::numeric_limits<float>::infinity();
}

// insert value x at position 3 and 7
inline __m256 insert_float_min_max(__m256 v, float x) {
  __m256 min_max_broadcast = _mm256_set1_ps(x);
  return _mm256_blend_ps(v, min_max_broadcast, 0b10001000);
}

/*
 * input (x1, y1, z1, minT, x2, y2, z2, minT). returns max(x1, y1, z1, minT),
 * max(x2, y2, z2, minT)
 */
inline std::pair<float, float> horizontal_max_128_lane(__m256 x) {
  // shuffle each lane to (z, minT, x, x)
  __m256 shuffle1 = _mm256_shuffle_ps(x, x, _MM_SHUFFLE(0, 0, 3, 2));

  // each lane result (max(x,z), max(y, t), max(x, z), max(x,t))
  __m256 max1 = _mm256_max_ps(x, shuffle1);

  // each lane let max1 ( m0, m1, m2, m3 ). then shuffle2 is (m1, m0, m0, m0)
  __m256 shuffle2 = _mm256_shuffle_ps(max1, max1, _MM_SHUFFLE(0, 0, 0, 1));

  // each lane's least significant element will be max(m0, m1). i.e max(max(x,z), max(y, t))
  __m256 max2 = _mm256_max_ps(max1, shuffle2);

  // get max for ray 1
  float tBoxMin1 = _mm256_cvtss_f32(max2);
  // get max for ray 2
  float tBoxMin2 = _mm_cvtss_f32(_mm256_extractf128_ps(max2, 1));

  return std::make_pair(tBoxMin1, tBoxMin2);
}

/*
 * input (x1, y1, z1, maxT, x2, y2, z2, maxT). returns min(x1, y1, z1, maxT),
 * min(x2, y2, z2, maxT)
 * same logic as horizontal_max_128_lane just using min operations
 */
inline std::pair<float, float> horizontal_min_128_lane(__m256 x) {
  __m256 shuffle1 = _mm256_shuffle_ps(x, x, _MM_SHUFFLE(0, 0, 3, 2));
  __m256 min1 = _mm256_min_ps(x, shuffle1);
  __m256 shuffle2 = _mm256_shuffle_ps(min1, min1, _MM_SHUFFLE(0, 0, 0, 1));
  __m256 min2 = _mm256_min_ps(min1, shuffle2);

  float tBoxMax1 = _mm256_cvtss_f32(min2);
  float tBoxMax2 = _mm_cvtss_f32(_mm256_extractf128_ps(min2, 1));

  return std::make_pair(tBoxMax1, tBoxMax2);
}

// aabb mins and maxes are given array of floats. Result is given in t1 and t2
// used slab_intersect_aabb_array function as basis for simd
// same function as ray_1aabb_slab but loads both sibling AABBs
inline std::pair<float, float> ray_2aabb_slab(const float* mins, const float* maxs,
                                              const __m256& ray_o, const __m256& ray_dir_inv,
                                              const Ray& r) {
  // (bboxes_mins - ray.origin) * invRayDir
  __m256 bbox_min_vals = _mm256_loadu_ps(mins);
  bbox_min_vals = _mm256_permutevar8x32_ps(bbox_min_vals, _mm256_set_epi32(6, 5, 4, 3, 7, 2, 1, 0));

  bbox_min_vals = _mm256_mul_ps(_mm256_sub_ps(bbox_min_vals, ray_o), ray_dir_inv);

  // (bboxes_maxes - ray.origin) * invRayDir
  __m256 bbox_max_vals = _mm256_loadu_ps(maxs);
  bbox_max_vals = _mm256_permutevar8x32_ps(bbox_max_vals, _mm256_set_epi32(6, 5, 4, 3, 7, 2, 1, 0));

  bbox_max_vals = _mm256_mul_ps(_mm256_sub_ps(bbox_max_vals, ray_o), ray_dir_inv);

  // element wise min and max, equivalent of tMins and tMaxes
  __m256 min_elements = _mm256_min_ps(bbox_min_vals, bbox_max_vals);
  __m256 max_elements = _mm256_max_ps(bbox_min_vals, bbox_max_vals);

  // insert minT to min_elements
  min_elements = insert_float_min_max(min_elements, r.minT);

  // get tBoxMin for ray 1 and 2
  auto [tBoxMin1, tBoxMin2] = horizontal_max_128_lane(min_elements);

  // insert minT to min_elements
  max_elements = insert_float_min_max(max_elements, r.maxT);

  // get tBoxMin for ray 1 and 2
  auto [tBoxMax1, tBoxMax2] = horizontal_min_128_lane(max_elements);

  float t1 = (tBoxMin1 <= tBoxMax1) ? tBoxMin1 : std::numeric_limits<float>::infinity();
  float t2 = (tBoxMin2 <= tBoxMax2) ? tBoxMin2 : std::numeric_limits<float>::infinity();

  return std::make_pair(t1, t2);
}
#endif