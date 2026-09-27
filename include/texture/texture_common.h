#pragma once

#include <algorithm>
#include <fastgltf/types.hpp>

// effects the handling of texture UVs when out of [0, 1] range
enum class TextureWrappingMode { ClampToEdge, MirroredRepeat, Repeat };

inline TextureWrappingMode gltf_wrap_convert(const fastgltf::Wrap gltf_wrap) {
  switch (gltf_wrap) {
    case fastgltf::Wrap::Repeat:
      return TextureWrappingMode::Repeat;
    case fastgltf::Wrap::MirroredRepeat:
      return TextureWrappingMode::MirroredRepeat;
    case fastgltf::Wrap::ClampToEdge:
      return TextureWrappingMode::ClampToEdge;
    default:
      return TextureWrappingMode::Repeat;
  }
}

inline float fast_clampf(float value, float min, float max) {
  return std::min(max, std::max(min, value));
}

inline int fast_clampi(int value, int min, int max) { return std::min(max, std::max(min, value)); }

inline float handle_wrapping(float coord, TextureWrappingMode mode) {
  switch (mode) {
    case TextureWrappingMode::ClampToEdge:
      return fast_clampf(coord, 0.f, 1.f);

    case TextureWrappingMode::Repeat: {
      float fraction = coord - static_cast<int>(coord);
      return fraction + std::signbit(fraction);
    }

    case TextureWrappingMode::MirroredRepeat: {
      int int_part = static_cast<int>(coord);
      float fraction = coord - int_part;
      int_part -= std::signbit(fraction);

      return (std::abs(int_part) % 2) - fraction + std::signbit(fraction);
    }

    default:
      return fast_clampf(coord, 0.f, 1.f);
  }
}