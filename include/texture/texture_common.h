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

inline float handle_wrapping(float coord, TextureWrappingMode mode) {
  switch (mode) {
    case TextureWrappingMode::ClampToEdge:
      return std::clamp(coord, 0.f, 1.f);

    case TextureWrappingMode::Repeat: {
      float fraction = coord - static_cast<int>(coord);

      if (std::signbit(fraction))
        return 1.f + fraction;
      else
        return fraction;
    }

    case TextureWrappingMode::MirroredRepeat: {
      int int_part = static_cast<int>(coord);
      float fraction = coord - int_part;

      if (std::signbit(fraction)) {
        if (int_part % 2)
          return std::fabs(fraction);
        else
          return 1.f + fraction;

      } else
        return fraction;
    }

    default:
      return std::clamp(coord, 0.f, 1.f);
  }
}