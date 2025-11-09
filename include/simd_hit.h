#pragma once

#ifdef __AVX2__
#include <hit_utils.h>
#include <immintrin.h>

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
inline void ray_1aabb_slab(const float* mins, const float* maxs, const __m256& ray_o,
                           const __m256& ray_dir_inv, const Ray& r, float& t1) {
  // (bboxes_mins - ray.origin) * invRayDir
  __m128 bbox_min_vals = _mm_load_ps(mins);
  bbox_min_vals = _mm_mul_ps(_mm_sub_ps(bbox_min_vals, _mm256_castps256_ps128(ray_o)),
                             _mm256_castps256_ps128(ray_dir_inv));

  // (bboxes_maxes - ray.origin) * invRayDir
  __m128 bbox_max_vals = _mm_load_ps(maxs);
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
    t1 = tBoxMin;
  else
    t1 = std::numeric_limits<float>::infinity();
}

// aabb mins and maxes are given array of floats. Result is given in t1 and t2
// used slab_intersect_aabb_array function as basis for simd
// same function as ray_1aabb_slab but loads both sibling AABBs
inline void ray_2aabb_slab(const float* mins, const float* maxs, const __m256& ray_o,
                           const __m256& ray_dir_inv, const Ray& r, float& t1, float& t2) {
  // (bboxes_mins - ray.origin) * invRayDir
  __m256 bbox_min_vals = _mm256_load_ps(mins);
  bbox_min_vals = _mm256_mul_ps(_mm256_sub_ps(bbox_min_vals, ray_o), ray_dir_inv);

  // (bboxes_maxes - ray.origin) * invRayDir
  __m256 bbox_max_vals = _mm256_load_ps(maxs);
  bbox_max_vals = _mm256_mul_ps(_mm256_sub_ps(bbox_max_vals, ray_o), ray_dir_inv);

  // element wise min and max, equivalent of tMins and tMaxes
  __m256 min_elements = _mm256_min_ps(bbox_min_vals, bbox_max_vals);
  __m256 max_elements = _mm256_max_ps(bbox_min_vals, bbox_max_vals);

  // get tBoxMin for AABB 1
  // ******************************
  // lower 128 bits for AABB 1
  __m128 part = _mm256_castps256_ps128(min_elements);

  // overwrite upper most value with r.minT. as that contains x value for ray 2
  __m128 ray_limit_val = _mm_set_ss(r.minT);
  __m128 min_max_op = _mm_insert_ps(part, ray_limit_val, 0b00'11'00'00);

  float tBoxMin1 = horizontal_max_128(min_max_op);
  // ******************************

  // get tBoxMin for AABB 2
  // ******************************
  // move floats at 3, 4, 5 to position 0, 1, 2
  min_elements = _mm256_permutevar8x32_ps(min_elements, _mm256_set_epi32(5, 5, 5, 5, 5, 5, 4, 3));

  // overwrite upper most value with r.minT
  part = _mm256_castps256_ps128(min_elements);
  min_max_op = _mm_insert_ps(part, ray_limit_val, 0b00'11'00'00);

  float tBoxMin2 = horizontal_max_128(min_max_op);
  // ******************************

  // get tBoxMax for AABB 1
  // ******************************
  // lower 128 bits for AABB 1
  part = _mm256_castps256_ps128(max_elements);

  // overwrite upper most value with r.maxT
  ray_limit_val = _mm_set_ss(r.maxT);
  min_max_op = _mm_insert_ps(part, ray_limit_val, 0b00'11'00'00);

  float tBoxMax1 = horizontal_min_128(min_max_op);
  // ******************************

  // get tBoxMax for AABB 2
  // ******************************
  // move floats at 3, 4, 5 to position 0, 1, 2
  max_elements = _mm256_permutevar8x32_ps(max_elements, _mm256_set_epi32(5, 5, 5, 5, 5, 5, 4, 3));

  // overwrite upper most value with r.maxT
  part = _mm256_castps256_ps128(max_elements);
  min_max_op = _mm_insert_ps(part, ray_limit_val, 0b00'11'00'00);
  // ******************************

  float tBoxMax2 = horizontal_min_128(min_max_op);

  if (tBoxMin1 <= tBoxMax1)
    t1 = tBoxMin1;
  else
    t1 = std::numeric_limits<float>::infinity();

  if (tBoxMin2 <= tBoxMax2)
    t2 = tBoxMin2;
  else
    t2 = std::numeric_limits<float>::infinity();
}
#endif