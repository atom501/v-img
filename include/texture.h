#pragma once

#include <hit_utils.h>
#include <ray.h>

#include <algorithm>
#include <filesystem>
#include <utility>
#include <vector>

#include "glm/common.hpp"
#include "glm/vec2.hpp"
#include "glm/vec3.hpp"

class Texture {
public:
  Texture() = default;
  ~Texture() = default;

  virtual glm::vec3 col_at_uv(const glm::vec3& ray_in_dir, const RayCone& cone,
                              const HitInfo& surf_hit) const
      = 0;
};

class ConstColor : public Texture {
private:
  glm::vec3 albedo;

public:
  ConstColor(const glm::vec3& albedo) : albedo(albedo) {}
  ~ConstColor() = default;

  glm::vec3 col_at_uv(const glm::vec3& ray_in_dir, const RayCone& cone,
                      const HitInfo& surf_hit) const override {
    return albedo;
  }
};

class Checkerboard : public Texture {
private:
  uint32_t width;
  uint32_t height;
  glm::vec3 col_a;
  glm::vec3 col_b;

public:
  Checkerboard(uint32_t width, uint32_t height, const glm::vec3& col_a, const glm::vec3& col_b)
      : width(width), height(height), col_a(col_a), col_b(col_b) {}
  ~Checkerboard() = default;

  glm::vec3 col_at_uv(const glm::vec3& ray_in_dir, const RayCone& cone,
                      const HitInfo& surf_hit) const override {
    uint32_t u_board = std::floor(surf_hit.uv[0] * width);
    uint32_t v_board = std::floor(surf_hit.uv[1] * height);

    if ((u_board + v_board) % 2 == 0)
      return col_a;
    else
      return col_b;
  }
};

class ImageTexture : public Texture {
private:
  uint32_t width;
  uint32_t height;
  std::vector<std::vector<glm::vec3>> mipmap;  // flattened size width * height. Top left corner is
                                               // 0,0 index. mipmap level 0 is original image
public:
  ImageTexture() = default;
  ~ImageTexture() = default;

  ImageTexture(const std::vector<glm::vec3>& image, uint32_t width, uint32_t height);

  glm::vec3 col_at_uv(const glm::vec3& ray_in_dir, const RayCone& cone,
                      const HitInfo& surf_hit) const override;

  void debug_mipmaps_to_file();

private:
  float compute_texture_LOD(const glm::vec3& ray_dir, const RayCone& cone,
                            const HitInfo& surf) const {
    float lambda = 0.5f * std::log2((surf.tex_coord_area) / surf.primitive_area);
    lambda += std::log2(std::abs(cone.cone_width) / std::abs(glm::dot(ray_dir, surf.hit_n_g)));
    lambda += 0.5f * log2(width * height);
    return lambda;
  }

  // return color for uv on given mipmap level. applies bilinear filtering
  glm::vec3 col_at_uv_mipmap(int mipmap_level, const glm::vec2& uv) const;
};

ImageTexture load_imagetexture(const std::filesystem::path& ImageTexture_file);