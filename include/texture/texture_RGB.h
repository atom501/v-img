#pragma once

#include <hit_utils.h>
#include <ray.h>
#include <texture/texture_common.h>

#include <algorithm>
#include <filesystem>
#include <utility>
#include <vector>

#include "fastgltf/types.hpp"
#include "glm/common.hpp"
#include "glm/vec2.hpp"
#include "glm/vec3.hpp"

// None means loaded as r,g,b with each channel 0 to 255. transform according to need
enum class TextureType { None, Image, Normals, MetallicRoughness };

// fmt formater for TextureType
inline auto format_as(TextureType t) {
  switch (t) {
    case TextureType::None:
      return "TextureType::None";
    case TextureType::Image:
      return "TextureType::Image";
    case TextureType::Normals:
      return "TextureType::Normals";
    case TextureType::MetallicRoughness:
      return "TextureType::MetallicRoughness";
  }
  return "unknown";
};

class TextureRGB {
public:
  TextureRGB() = default;
  virtual ~TextureRGB() = default;

  virtual glm::vec3 col_at_ray_hit(const glm::vec3& ray_in_dir, const RayCone& cone,
                                   const HitInfo& surf_hit) const
      = 0;
};

class ConstColor : public TextureRGB {
private:
  glm::vec3 albedo;

public:
  ConstColor(const glm::vec3& albedo) : albedo(albedo) {}
  ~ConstColor() = default;

  glm::vec3 col_at_ray_hit(const glm::vec3& ray_in_dir, const RayCone& cone,
                           const HitInfo& surf_hit) const override {
    return albedo;
  }
};

class Checkerboard : public TextureRGB {
private:
  uint32_t width;
  uint32_t height;
  glm::vec3 col_a;
  glm::vec3 col_b;

public:
  Checkerboard(uint32_t width, uint32_t height, const glm::vec3& col_a, const glm::vec3& col_b)
      : width(width), height(height), col_a(col_a), col_b(col_b) {}
  ~Checkerboard() = default;

  glm::vec3 col_at_ray_hit(const glm::vec3& ray_in_dir, const RayCone& cone,
                           const HitInfo& surf_hit) const override {
    uint32_t u_board = std::floor(surf_hit.uv[0] * width);
    uint32_t v_board = std::floor(surf_hit.uv[1] * height);

    if ((u_board + v_board) % 2 == 0)
      return col_a;
    else
      return col_b;
  }
};

class ImageTexture : public TextureRGB {
public:
  uint32_t width;
  uint32_t height;
  /*
   * flattened size width * height. Top left corner is
   * 0,0 index. mipmap level 0 is original image
   */
  std::vector<std::vector<glm::vec3>> mipmap;

  TextureWrappingMode u_wrapping_mode = TextureWrappingMode::Repeat;
  TextureWrappingMode v_wrapping_mode = TextureWrappingMode::Repeat;

public:
  ImageTexture() = default;
  ~ImageTexture() = default;

  ImageTexture(const std::vector<glm::vec3>& image, uint32_t width, uint32_t height,
               TextureWrappingMode u_wrapping_mode, TextureWrappingMode v_wrapping_mode);

  // make image texture without making mipmaps
  ImageTexture(const std::vector<std::vector<glm::vec3>>& image, uint32_t width, uint32_t height,
               TextureWrappingMode u_wrapping_mode, TextureWrappingMode v_wrapping_mode)
      : width(width),
        height(height),
        mipmap(image),
        u_wrapping_mode(u_wrapping_mode),
        v_wrapping_mode(v_wrapping_mode) {};

  // color when hitting a surface
  glm::vec3 col_at_ray_hit(const glm::vec3& ray_in_dir, const RayCone& cone,
                           const HitInfo& surf_hit) const override;
  // color of background
  glm::vec3 col_mipmap_interpolate(float lambda, const glm::vec2& uv) const;

  // return color for uv on given mipmap level. applies bilinear filtering
  glm::vec3 col_at_uv_mipmap(int mipmap_level, const glm::vec2& uv) const;

  // only for normal maps. normalized after bilinear filtering
  glm::vec3 get_normal(const glm::vec2& uv) const;

  void debug_mipmaps_to_file();

  // converts sRGB [0, 255] to linear space [0, 1]
  static void convert_sRGB_to_linear(std::vector<glm::vec3>& image);

  /*
   * converts RGB [0, 255] to tangent space normal vectors. Z is perpedicular to the surface.
   * Same as gltf spec: red [0.0 .. 1.0] to X [-1 .. 1], green [0.0 .. 1.0] to Y
   * [-1 .. 1], blue (0.5 .. 1.0] maps to Z (0 .. 1]
   * scale scales X and Y components. Vector is normalized after scaling
   */
  static void convert_RGB_to_normal(std::vector<glm::vec3>& image, float scale);

private:
  float compute_texture_LOD(const glm::vec3& ray_dir, const RayCone& cone,
                            const HitInfo& surf) const {
    float lambda = 0.5f * std::log2((surf.tex_coord_area) / surf.primitive_area);
    lambda += std::log2(std::abs(cone.cone_width) / std::abs(glm::dot(ray_dir, surf.hit_n_g)));
    lambda += 0.5f * log2(width * height);

    if (std::isnan(lambda)) {
      return 0.f;
    } else {
      return lambda;
    }
  }
};

ImageTexture load_imagetexture(const std::filesystem::path& ImageTexture_file);