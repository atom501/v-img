#pragma once

#include <material/material.h>
#include <rng/sampling.h>

inline float G_w(const glm::vec3& w, float alphax, float alphay, const ONB& frame) {
  const glm::vec3 w_local = project_onto_onb(frame, w);
  float vec_alpha
      = ((w_local.x * alphax) * (w_local.x * alphax) + (w_local.y * alphay) * (w_local.y * alphay))
        / (w_local.z * w_local.z);
  float caret = (std::sqrt(1. + (vec_alpha)) - 1.) / 2.;

  return 1. / (1. + caret);
}

inline glm::vec3 anisotropic_sample_visible_normals(const glm::vec3& local_dir_in, float alphax,
                                                    float alphay, pcg32_random_t& pcg_rng) {
  // The incoming direction is in the "ellipsodial configuration" in Heitz's paper

  float sign = 1.f;
  glm::vec3 local_dir_in_top = local_dir_in;
  if (local_dir_in.z < 0.f) {
    // Ensure the input is on top of the surface.
    sign = -1.f;
    local_dir_in_top = -local_dir_in_top;
  }

  // Transform the incoming direction to the "hemisphere configuration".
  glm::vec3 hemi_dir_in = glm::normalize(
      glm::vec3{alphax * local_dir_in_top.x, alphay * local_dir_in_top.y, local_dir_in_top.z});

  float randx = rand_float(pcg_rng);
  float randy = rand_float(pcg_rng);
  // Parameterization of the projected area of a hemisphere.
  // First, sample a disk.
  float r = std::sqrt(randx);
  float phi = 2 * std::numbers::pi * randy;
  float t1 = r * cos(phi);
  float t2 = r * sin(phi);
  // Vertically scale the position of a sample to account for the projection.
  float s = (1 + hemi_dir_in.z) / 2;
  t2 = (1 - s) * sqrt(1 - t1 * t1) + s * t2;
  // Point in the disk space
  glm::vec3 disk_N{t1, t2, std::sqrt(std::max(0.f, 1 - t1 * t1 - t2 * t2))};

  // Reprojection onto hemisphere -- we get our sampled normal in hemisphere space.
  ONB hemi_frame = init_onb(hemi_dir_in);
  glm::vec3 hemi_N = xform_with_onb(hemi_frame, disk_N);

  // Transforming the normal back to the ellipsoid configuration
  return sign
         * glm::normalize(glm::vec3{alphax * hemi_N.x, alphay * hemi_N.y, std::max(0.f, hemi_N.z)});
}