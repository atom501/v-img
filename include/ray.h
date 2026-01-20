#pragma once

#include <limits>
#include <optional>
#include <utility>

#include "glm/mat4x4.hpp"
#include "glm/vec3.hpp"

// sources: Ray Tracing Gems 1: CHAPTER 20, Ray Tracing Gems 2: CHAPTER 10
struct RayCone {
  float cone_width;
  float spread_angle;  // angle between center axis and cone edge
};

class Ray {
public:
  glm::vec3 dir = glm::vec3(1.0f);
  glm::vec3 o = glm::vec3(0.0f);

  float minT = 0.0001f;
  float maxT = std::numeric_limits<float>::infinity();

  RayCone ray_cone;

public:
  Ray(const glm::vec3& o, const glm::vec3& d) : dir(d), o(o) {}
  Ray(const glm::vec3& o, const glm::vec3& d, const RayCone& ray_cone)
      : dir(d), o(o), ray_cone(ray_cone) {}
  Ray() = default;
  ~Ray() = default;

  glm::vec3 at(float t) { return o + t * dir; }

  // transform the current ray
  void xform_ray(const glm::mat4& xform) {
    dir = glm::vec3(xform * glm::vec4(dir, 0.0f));
    glm::vec4 temp_o = xform * glm::vec4(o, 1.0f);
    temp_o /= temp_o[3];
    o = glm::vec3(temp_o);
  }
};

inline RayCone raycone_for_primary_ray(float vfov, uint32_t pixel_height) {
  float spread_angle = std::atan(2.f * (std::tan(vfov / 2.f)) / static_cast<float>(pixel_height));
  float cone_width = 0.f;
  return RayCone{cone_width, spread_angle};
}

static inline float float_sign(float in) { return in > 0.f ? 1.f : -1.f; }

inline float spread_angle_from_curvature(float mean_curvature, float rayConeWidth,
                                         const glm::vec3& rayDir, const glm::vec3& normal) {
  float dn = -glm::dot(rayDir, normal);
  dn = std::abs(dn) < 1.0e-5 ? float_sign(dn) * 1.0e-5 : dn;

  float deltaPhi = (mean_curvature * rayConeWidth / dn);
  float surfaceSpreadAngle = deltaPhi;

  return surfaceSpreadAngle;
}

inline RayCone propagate_reflect_cone(const RayCone& cone, float surface_spread_angle,
                                      float hit_dist) {
  float new_cone_width = std::abs(cone.spread_angle * hit_dist + cone.cone_width);
  float new_spread_angle = cone.spread_angle + surface_spread_angle;
  return RayCone{new_cone_width, new_spread_angle};
}

static inline std::optional<glm::vec2> refract_with_TIR2D(const glm::vec2& rayDir,
                                                          const glm::vec2& normal, float eta) {
  float NdotD = glm::dot(normal, rayDir);
  float k = 1.0f - eta * eta * (1.0f - NdotD * NdotD);
  if (k < 0.0f) {
    return std::nullopt;
  } else {
    return rayDir * eta - normal * (eta * NdotD + std::sqrt(k));
  }
}

static inline std::pair<glm::vec2, glm::vec2> rotate2DPlusMinus(const glm::vec2& vec, float angle) {
  float c = cos(angle);
  float s = sin(angle);
  float cx = c * vec.x;
  float sy = s * vec.y;
  float sx = s * vec.x;
  float cy = c * vec.y;

  // return (Rotate +angle, Rotate -angle)
  return std::make_pair(glm::vec2(cx - sy, +sx + cy), glm::vec2(cx + sy, -sx + cy));
}

static inline glm::vec2 orthogonal(const glm::vec2& vec) { return glm::vec2(-vec.y, vec.x); }

inline RayCone propagate_refract_cone(const RayCone& rayCone, glm::vec3 ray_in_dir,
                                      glm::vec3 hitPoint, float surface_spread_angle, float eta,
                                      glm::vec3 refractedRayDir) {
  // re-calculated incase half-vector is used
  glm::vec3 normal
      = -(eta * refractedRayDir + ray_in_dir) / glm::length(eta * refractedRayDir + ray_in_dir);

  // We have refractedRayDir, which is the direction of the refracted ray cone,
  // but we also need the rayCone.width and the rayCone.spreadAngle. These are computed in 2D,
  // with xAxis and yAxis as the 3D axes. hitPoint is the origin of this 2D coordinate system.
  glm::vec3 xAxis = glm::normalize(ray_in_dir - normal * glm::dot(normal, ray_in_dir));
  glm::vec3 yAxis = normal;

  glm::vec2 refractedDir2D = glm::vec2(glm::dot(refractedRayDir, xAxis),
                                       glm::dot(refractedRayDir, yAxis));  // Project to 2D.
  glm::vec2 incidentDir2D
      = glm::vec2(glm::dot(ray_in_dir, xAxis), glm::dot(ray_in_dir, yAxis));  // Project to 2D.
  glm::vec2 incidentDirOrtho2D = orthogonal(incidentDir2D);

  float widthSign = rayCone.cone_width > 0.0f ? 1.0f : -1.0f;

  // Upper (_u) and lower (_l) line of ray cone in 2D.
  auto [incidentDir2D_u, incidentDir2D_l]
      = rotate2DPlusMinus(incidentDir2D, rayCone.spread_angle * widthSign * 0.5f);

  glm::vec2 tu = +incidentDirOrtho2D * rayCone.cone_width
                 * 0.5f;  // Top, upper point on the incoming ray cone (in 2D).
  glm::vec2 tl = -tu;     // Top, lower point on the incoming ray cone (in 2D).

  float hitPoint_u_x = tu.x + incidentDir2D_u.x * (-tu.y / incidentDir2D_u.y);
  float hitPoint_l_x = tl.x + incidentDir2D_l.x * (-tl.y / incidentDir2D_l.y);

  float normalSign = hitPoint_u_x > hitPoint_l_x ? +1.0f : -1.0f;

  glm::vec2 normal2D = glm::vec2(0.0f, 1.0f);

  auto [normal2D_u, normal2D_l]
      = rotate2DPlusMinus(normal2D, -surface_spread_angle * normalSign * 0.5f);

  // Refract in 2D.
  std::optional<glm::vec2> refractedDir2D_u = refract_with_TIR2D(incidentDir2D_u, normal2D_u, eta);
  if (!refractedDir2D_u.has_value()) {
    refractedDir2D_u = incidentDir2D_u - normal2D_u * dot(normal2D_u, incidentDir2D_u);
    refractedDir2D_u = glm::normalize(refractedDir2D_u.value());
  }

  std::optional<glm::vec2> refractedDir2D_l = refract_with_TIR2D(incidentDir2D_l, normal2D_l, eta);
  if (!refractedDir2D_l.has_value()) {
    refractedDir2D_l = incidentDir2D_l - normal2D_l * dot(normal2D_l, incidentDir2D_l);
    refractedDir2D_l = glm::normalize(refractedDir2D_l.value());
  }

  glm::vec2& refractedDir2D_u_val = refractedDir2D_u.value();
  glm::vec2& refractedDir2D_l_val = refractedDir2D_l.value();

  float signA = (refractedDir2D_u_val.x * refractedDir2D_l_val.y
                 - refractedDir2D_u_val.y * refractedDir2D_l_val.x)
                            * normalSign
                        < 0.0f
                    ? +1.0f
                    : -1.0f;
  float spreadAngle = std::acos(glm::dot(refractedDir2D_u_val, refractedDir2D_l_val)) * signA;

  if (std::isnan(spreadAngle)) {
    spreadAngle = 0.f;
  }

  // Now compute the width of the refracted cone.
  glm::vec2 refractDirOrtho2D = orthogonal(refractedDir2D);

  // Intersect line (0,0) + t * refractDirOrtho2D with the line: hitPoint_u + s * refractedDir2D_u,
  // but optimized since hitPoint_ul.y=0.
  float width = (-hitPoint_u_x * refractedDir2D_u_val.y)
                / dot(refractDirOrtho2D, orthogonal(refractedDir2D_u_val));
  // Intersect line (0,0) + t * refractDirOrtho2D with the line: hitPoint_l + s * refractedDir2D_l.
  width += (hitPoint_l_x * refractedDir2D_l_val.y)
           / dot(refractDirOrtho2D, orthogonal(refractedDir2D_l_val));

  return RayCone{width, spreadAngle};
}
