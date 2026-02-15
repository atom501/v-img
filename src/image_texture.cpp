#include <texture.h>
#define STB_IMAGE_IMPLEMENTATION
#include <color_utils.h>
#include <stb_image_write.h>
#include <tinyexr.h>

#include "fmt/core.h"
#include "stb_image.h"

// loading image from disk and constructing ImageTexture from it
ImageTexture load_imagetexture(const std::filesystem::path& ImageTexture_file) {
  int width, height, channels;
  std::vector<glm::vec3> image;

  std::string env_type = ImageTexture_file.extension().generic_string();

  // LoadEXR and stbi_loadf will give top left corner as pixel 0, 0
  if (env_type == ".exr") {
    float* out;  // width * height * RGBA
    const char* err = nullptr;

    int ret = LoadEXR(&out, &width, &height, ImageTexture_file.generic_string().c_str(), &err);

    if (ret != TINYEXR_SUCCESS) {
      if (err) {
        fprintf(stderr, "ERR : %s\n", err);
        FreeEXRErrorMessage(err);
      }
    } else {
      // copy image data to vector. ignore alpha channel for now
      image = std::vector<glm::vec3>(width * height);

      for (size_t i = 0; i < width * height; i++) {
        image[i] = glm::vec3(out[i * 4], out[i * 4 + 1], out[i * 4 + 2]);
      }

      free(out);  // release memory of image data
    }
  } else {
    // already gamma corrected
    float* image_array
        = stbi_loadf(ImageTexture_file.string().c_str(), &width, &height, &channels, STBI_rgb);

    image = std::vector<glm::vec3>(width * height);

    size_t arr_index = 0;
    for (size_t i = 0; i < width * height; i++) {
      image[i].r = image_array[arr_index++];
      image[i].g = image_array[arr_index++];
      image[i].b = image_array[arr_index++];
    }
  }

  return ImageTexture(image, width, height);
}

ImageTexture::ImageTexture(const std::vector<glm::vec3>& image, uint32_t width, uint32_t height) {
  // original image is given as parameter
  ImageTexture::mipmap.push_back(image);
  ImageTexture::width = width;
  ImageTexture::height = height;

  // fmt::println("mipmap level 0. Width {}, Height {}", width, height);

  static constexpr int max_mipmap_level = 15;
  uint32_t size = std::min(width, height);

  // make mipmap by applying filter to each level
  int num_levels = std::min((int)std::ceil(std::log2(float(size))), max_mipmap_level);

  uint32_t prev_width = width;
  uint32_t prev_height = height;

  for (size_t l = 1; l < num_levels; l++) {
    const std::vector<glm::vec3>& prev_img = mipmap[l - 1];
    int next_w = std::max(prev_width / 2u, 1u);
    int next_h = std::max(prev_height / 2u, 1u);

    // fmt::println("mipmap level {}. Width {}, Height {}", l, next_w, next_h);

    std::vector<glm::vec3> new_level(next_w * next_h);

    uint32_t old_size = prev_width * prev_height;

    for (size_t y = 0; y < next_h; y++) {
      for (size_t x = 0; x < next_w; x++) {
        glm::vec3 sum = glm::vec3(0.f);

        // downsampling filter. source:
        // https://bartwronski.com/2022/03/07/fast-gpu-friendly-antialiasing-downsampling-filter/

        glm::vec2 inv_res = glm::vec2(1.f / prev_width, 1.f / prev_height);
        glm::vec2 curr_uv = glm::vec2(2 * x, 2 * y) * inv_res;
        sum += 0.37487566f
               * col_at_uv_mipmap(l - 1, curr_uv + glm::vec2(-0.75777, -0.75777) * inv_res);
        sum += 0.37487566f
               * col_at_uv_mipmap(l - 1, curr_uv + glm::vec2(0.75777, -0.75777) * inv_res);
        sum += 0.37487566f
               * col_at_uv_mipmap(l - 1, curr_uv + glm::vec2(0.75777, 0.75777) * inv_res);
        sum += 0.37487566f
               * col_at_uv_mipmap(l - 1, curr_uv + glm::vec2(-0.75777, 0.75777) * inv_res);

        sum += -0.12487566f * col_at_uv_mipmap(l - 1, curr_uv + glm::vec2(-2.907, 0.0) * inv_res);
        sum += -0.12487566f * col_at_uv_mipmap(l - 1, curr_uv + glm::vec2(2.907, 0.0) * inv_res);
        sum += -0.12487566f * col_at_uv_mipmap(l - 1, curr_uv + glm::vec2(0.0, -2.907) * inv_res);
        sum += -0.12487566f * col_at_uv_mipmap(l - 1, curr_uv + glm::vec2(0.0, 2.907) * inv_res);

        if (sum.x < 0) sum.x = 0.f;
        if (sum.y < 0) sum.y = 0.f;
        if (sum.z < 0) sum.z = 0.f;

        new_level[x + y * next_w] = sum;
      }
    }

    mipmap.push_back(new_level);

    prev_width = next_w;
    prev_height = next_h;
  }
}

glm::vec3 ImageTexture::col_at_uv_mipmap(int mipmap_level, const glm::vec2& uv) const {
  uint32_t mip_w = std::max(ImageTexture::width >> mipmap_level, 1u);
  uint32_t mip_h = std::max(ImageTexture::height >> mipmap_level, 1u);

  // get value on the mipmap level
  float pixel_u = uv[0] * mip_w;
  float pixel_v = uv[1] * mip_h;

  // get pixel value using sampling
  int curr_x = std::clamp(static_cast<int>(pixel_u), 0, static_cast<int>(mip_w) - 1);
  int curr_y = std::clamp(static_cast<int>(pixel_v), 0, static_cast<int>(mip_h) - 1);

  int next_x = std::clamp(curr_x + 1, 0, static_cast<int>(mip_w) - 1);
  int next_y = std::clamp(curr_y + 1, 0, static_cast<int>(mip_h) - 1);

  float x_fraction = pixel_u - curr_x;
  float y_fraction = pixel_v - curr_y;

  glm::vec3 x0 = ImageTexture::mipmap[mipmap_level][curr_x + curr_y * mip_w];
  glm::vec3 x1 = ImageTexture::mipmap[mipmap_level][next_x + curr_y * mip_w];

  glm::vec3 a = glm::mix(x0, x1, glm::vec3(x_fraction));

  glm::vec3 y0 = ImageTexture::mipmap[mipmap_level][curr_x + next_y * mip_w];
  glm::vec3 y1 = ImageTexture::mipmap[mipmap_level][next_x + next_y * mip_w];

  glm::vec3 b = glm::mix(y0, y1, glm::vec3(x_fraction));

  return glm::mix(a, b, glm::vec3(y_fraction));
}

glm::vec3 ImageTexture::col_at_ray_hit(const glm::vec3& ray_in_dir, const RayCone& cone,
                                       const HitInfo& surf_hit) const {
  float lambda;
  if constexpr (DEBUG_MIPMAPS) {
    lambda = 0;
  } else {
    // calculate mipmap level. subtract so it isn't too aggressive
    lambda = compute_texture_LOD(ray_in_dir, cone, surf_hit) - 2.f;
  }

  lambda = std::clamp(lambda, 0.f, static_cast<float>(mipmap.size() - 1));

  int mipmap_level0 = std::clamp(static_cast<int>(std::floor(lambda)), static_cast<int>(0),
                                 static_cast<int>(mipmap.size() - 1));
  int mipmap_level1
      = std::clamp(mipmap_level0 + 1, static_cast<int>(0), static_cast<int>(mipmap.size() - 1));

  float fraction = lambda - std::floor(lambda);

  glm::vec3 col0 = col_at_uv_mipmap(mipmap_level0, surf_hit.uv);
  glm::vec3 col1 = col_at_uv_mipmap(mipmap_level1, surf_hit.uv);

  return glm::mix(col0, col1, glm::vec3(fraction));
}

glm::vec3 ImageTexture::col_mipmap_interpolate(float lambda, const glm::vec2& uv) const {
  lambda = std::clamp(lambda, 0.f, static_cast<float>(mipmap.size() - 1));
  int mipmap_level0 = std::clamp(static_cast<int>(std::floor(lambda)), static_cast<int>(0),
                                 static_cast<int>(mipmap.size() - 1));
  int mipmap_level1
      = std::clamp(mipmap_level0 + 1, static_cast<int>(0), static_cast<int>(mipmap.size() - 1));

  float fraction = lambda - std::floor(lambda);

  glm::vec3 col0 = col_at_uv_mipmap(mipmap_level0, uv);
  glm::vec3 col1 = col_at_uv_mipmap(mipmap_level1, uv);

  return glm::mix(col0, col1, glm::vec3(fraction));
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

void ImageTexture::convert_sRGB_to_linear(std::vector<glm::vec3>& image) {
  for (glm::vec3& pixel : image) {
    pixel /= 255.f;
    pixel = pix_sRGB_to_linear(pixel);
  }
}

void ImageTexture::convert_RGB_to_normal(std::vector<glm::vec3>& image, float scale) {
  for (glm::vec3& normal : image) {
    normal /= 255.f;
    normal = (normal * 2.f) - glm::vec3(1.f);

    normal.x *= scale;
    normal.y *= scale;

    normal = glm::normalize(normal);
  }
}

glm::vec3 ImageTexture::get_normal(const glm::vec2& uv) const {
  return glm::normalize(col_at_uv_mipmap(0, uv));
}