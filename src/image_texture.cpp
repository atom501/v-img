#include <texture.h>
#define STB_IMAGE_IMPLEMENTATION
#include <stb_image_write.h>

#include "fmt/core.h"
#include "stb_image.h"

// loading image from disk and constructing ImageTexture from it
ImageTexture load_imagetexture(const std::filesystem::path& ImageTexture_file) {
  int width, height, channels;

  // already gamma corrected
  float* image_array
      = stbi_loadf(ImageTexture_file.string().c_str(), &width, &height, &channels, STBI_rgb);

  std::vector<glm::vec3> image(width * height);

  size_t arr_index = 0;
  for (size_t i = 0; i < width * height; i++) {
    image[i].r = image_array[arr_index++];
    image[i].g = image_array[arr_index++];
    image[i].b = image_array[arr_index++];
  }

  return ImageTexture(image, width, height);
}

ImageTexture::ImageTexture(const std::vector<glm::vec3>& image, uint32_t width, uint32_t height) {
  // original image is given as parameter
  ImageTexture::mipmap.push_back(image);
  ImageTexture::width = width;
  ImageTexture::height = height;

  fmt::println("mipmap level 0. Width {}, Height {}", width, height);

  static constexpr int max_mipmap_level = 8;
  uint32_t size = std::max(width, height);

  // make mipmap by box filtering each level
  int num_levels = std::min((int)std::ceil(std::log2(float(size)) + 1.f), max_mipmap_level);

  uint32_t prev_width = width;
  uint32_t prev_height = height;

  for (size_t l = 1; l < num_levels; l++) {
    const std::vector<glm::vec3>& prev_img = mipmap[l - 1];
    int next_w = std::max(prev_width / 2u, 1u);
    int next_h = std::max(prev_height / 2u, 1u);

    fmt::println("mipmap level {}. Width {}, Height {}", l, next_w, next_h);

    std::vector<glm::vec3> new_level(next_w * next_h);

    uint32_t old_size = prev_width * prev_height;

    for (size_t y = 0; y < next_h; y++) {
      for (size_t x = 0; x < next_w; x++) {
        glm::vec3 sum = glm::vec3(0.f);

        // if box filter is out of bound then pad with zeros
        size_t sum_index = (2 * x) + (2 * y * prev_width);
        sum += sum_index < old_size ? prev_img[sum_index] : glm::vec3(0.f);

        sum_index = (2 * x + 1) + (2 * y * prev_width);
        sum += sum_index < old_size ? prev_img[sum_index] : glm::vec3(0.f);

        sum_index = (2 * x) + (2 * y * prev_width + 1);
        sum += sum_index < old_size ? prev_img[sum_index] : glm::vec3(0.f);

        sum_index = (2 * x + 1) + (2 * y * prev_width + 1);
        sum += sum_index < old_size ? prev_img[sum_index] : glm::vec3(0.f);

        sum /= 4.f;

        new_level[x + y * next_w] = sum;
      }
    }

    mipmap.push_back(new_level);

    prev_width = next_w;
    prev_height = next_h;
  }
}

glm::vec3 ImageTexture::col_at_uv(const glm::vec2& uv) const {
  float pixel_u = uv[0] * width;
  float pixel_v = uv[1] * height;

  // get pixel value using sampling
  int curr_x = std::clamp(static_cast<int>(pixel_u), 0, static_cast<int>(width) - 1);
  int curr_y = std::clamp(static_cast<int>(pixel_v), 0, static_cast<int>(height) - 1);

  int next_x = std::clamp(curr_x + 1, 0, static_cast<int>(width) - 1);
  int next_y = std::clamp(curr_y + 1, 0, static_cast<int>(height) - 1);

  float x_fraction = pixel_u - curr_x;
  float y_fraction = pixel_v - curr_y;

  glm::vec3 x0 = ImageTexture::mipmap[0][curr_x + curr_y * width];
  glm::vec3 x1 = ImageTexture::mipmap[0][next_x + curr_y * width];

  glm::vec3 a = glm::mix(x0, x1, glm::vec3(x_fraction));

  glm::vec3 y0 = ImageTexture::mipmap[0][curr_x + next_y * width];
  glm::vec3 y1 = ImageTexture::mipmap[0][next_x + next_y * width];

  glm::vec3 b = glm::mix(y0, y1, glm::vec3(x_fraction));

  return glm::mix(a, b, glm::vec3(y_fraction));
}

// for debugging. writes all mipmaps as png
void ImageTexture::debug_mipmaps_to_file() {
  const std::vector<glm::vec3>& mipmap0 = ImageTexture::mipmap[0];

  uint8_t* pixels;
  pixels = new uint8_t[ImageTexture::width * ImageTexture::height * 3];

  int index = 0;
  for (size_t i = 0; i < mipmap0.size(); i++) {
    // if NaN, write magenta
    if (std::isnan(mipmap0[i].r) || std::isnan(mipmap0[i].g) || std::isnan(mipmap0[i].b)) {
      pixels[index++] = 255;
      pixels[index++] = 0;
      pixels[index++] = 255;
    } else {
      // multiply by 255.999 so rounding occurs correctly
      pixels[index++] = std::clamp(static_cast<int>(255.999 * std::sqrt(mipmap0[i][0])), 0, 255);
      pixels[index++] = std::clamp(static_cast<int>(255.999 * std::sqrt(mipmap0[i][1])), 0, 255);
      pixels[index++] = std::clamp(static_cast<int>(255.999 * std::sqrt(mipmap0[i][2])), 0, 255);
    }
  }

  stbi_write_png("mipmap0.png", ImageTexture::width, ImageTexture::height, 3, pixels,
                 ImageTexture::width * 3);

  fmt::println("written mipmap level 0. Width {}, Height {}", ImageTexture::width,
               ImageTexture::height);

  delete[] pixels;

  uint32_t prev_width = ImageTexture::width;
  uint32_t prev_height = ImageTexture::height;

  for (size_t l = 1; l < ImageTexture::mipmap.size(); l++) {
    int curr_w = std::max(prev_width / 2u, 1u);
    int curr_h = std::max(prev_height / 2u, 1u);

    const std::vector<glm::vec3>& curr_mip = ImageTexture::mipmap[l];

    pixels = new uint8_t[curr_w * curr_h * 3];

    int index = 0;
    for (size_t i = 0; i < curr_mip.size(); i++) {
      // if NaN, write magenta
      if (std::isnan(curr_mip[i].r) || std::isnan(curr_mip[i].g) || std::isnan(curr_mip[i].b)) {
        pixels[index++] = 255;
        pixels[index++] = 0;
        pixels[index++] = 255;
      } else {
        // multiply by 255.999 so rounding occurs correctly
        pixels[index++] = std::clamp(static_cast<int>(255.999 * std::sqrt(curr_mip[i][0])), 0, 255);
        pixels[index++] = std::clamp(static_cast<int>(255.999 * std::sqrt(curr_mip[i][1])), 0, 255);
        pixels[index++] = std::clamp(static_cast<int>(255.999 * std::sqrt(curr_mip[i][2])), 0, 255);
      }
    }

    stbi_write_png(fmt::format("mipmap{}.png", l).c_str(), curr_w, curr_h, 3, pixels, curr_w * 3);
    fmt::println("written mipmap level {}. Width {}, Height {}", l, curr_w, curr_h);

    prev_width = curr_w;
    prev_height = curr_h;

    delete[] pixels;
  }
}