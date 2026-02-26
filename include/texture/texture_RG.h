#pragma once

#include <texture/texture_common.h>

#include <vector>

#include "glm/common.hpp"
#include "glm/vec2.hpp"

// two channel texture
class TextureRG {
private:
  uint32_t width;
  uint32_t height;

  std::vector<glm::vec2> roughness_metallic_vec;

  TextureWrappingMode u_wrapping_mode = TextureWrappingMode::Repeat;
  TextureWrappingMode v_wrapping_mode = TextureWrappingMode::Repeat;

public:
  TextureRG(uint32_t width, uint32_t height, const std::vector<glm::vec2>& roughness_metallic_vec,
            TextureWrappingMode u_wrapping_mode, TextureWrappingMode v_wrapping_mode)
      : width(width),
        height(height),
        roughness_metallic_vec(roughness_metallic_vec),
        u_wrapping_mode(u_wrapping_mode),
        v_wrapping_mode(v_wrapping_mode) {};

  ~TextureRG() = default;

  glm::vec2 get_at_uv(glm::vec2 uv) const {
    float pixel_u = handle_wrapping(uv[0], TextureRG::u_wrapping_mode) * TextureRG::width;
    float pixel_v = handle_wrapping(uv[1], TextureRG::v_wrapping_mode) * TextureRG::height;

    // get pixel value using sampling
    int curr_x = std::clamp(static_cast<int>(pixel_u), 0, static_cast<int>(TextureRG::width) - 1);
    int curr_y = std::clamp(static_cast<int>(pixel_v), 0, static_cast<int>(TextureRG::height) - 1);

    int next_x = std::clamp(curr_x + 1, 0, static_cast<int>(TextureRG::width) - 1);
    int next_y = std::clamp(curr_y + 1, 0, static_cast<int>(TextureRG::height) - 1);

    float x_fraction = pixel_u - curr_x;
    float y_fraction = pixel_v - curr_y;

    glm::vec2 x0 = TextureRG::roughness_metallic_vec[curr_x + curr_y * TextureRG::width];
    glm::vec2 x1 = TextureRG::roughness_metallic_vec[next_x + curr_y * TextureRG::height];

    glm::vec2 a = glm::mix(x0, x1, glm::vec2(x_fraction));

    glm::vec2 y0 = TextureRG::roughness_metallic_vec[curr_x + next_y * TextureRG::width];
    glm::vec2 y1 = TextureRG::roughness_metallic_vec[next_x + next_y * TextureRG::height];

    glm::vec2 b = glm::mix(y0, y1, glm::vec2(x_fraction));

    return glm::mix(a, b, glm::vec2(y_fraction));
  }
};