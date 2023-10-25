#include <cstdint>
#include <iostream>
#include <vector>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <geometry/sphere.h>
#include <integrators.h>
#include <stb_image_write.h>
#include <tl_camera.h>
#include <tonemapper.h>

#include <glm/gtc/matrix_transform.hpp>

#include "glm/glm.hpp"

int main() {
  const int width = 1000;
  const int height = 1000;
#define CHANNEL_NUM 3

  auto test = glm::vec4(0.5f);
  glm::mat4 look = camToWorld(glm::vec3(0.0f, 0.0f, 3.0f), glm::vec3(0.0f, 0.0f, 0.0f),
                              glm::vec3(0.0f, 1.0f, 0.0f));
  TLCam camera(look, glm::ivec2(width, height), 90.0f);

  Sphere s(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), 1.0f);

  /*** NOTICE!! You have to use uint8_t array to pass in stb function  ***/
  // Because the size of color is normally 255, 8bit.
  // If you don't use this one, you will get a weird image.

  integrator_data data;
  data.resolution = glm::ivec2(width, height);
  data.samples = 100;

  std::vector<glm::vec3> acc_image = normal_integrator(data, camera, s);

  // TODO apply tone mapper
  // aces_approx(acc_image);

  uint8_t* pixels = new uint8_t[width * height * CHANNEL_NUM];

  // convert from [0,1] to [0,256]
  int index = 0;

  for (size_t i = 0; i < acc_image.size(); i++) {
    pixels[index++] = static_cast<int>(255.99 * acc_image[i][0]);
    pixels[index++] = static_cast<int>(255.99 * acc_image[i][1]);
    pixels[index++] = static_cast<int>(255.99 * acc_image[i][2]);
  }

  // if CHANNEL_NUM is 4, you can use alpha channel in png
  stbi_write_png("v_img.png", width, height, CHANNEL_NUM, pixels, width * CHANNEL_NUM);

  delete[] pixels;

  return 0;
}
