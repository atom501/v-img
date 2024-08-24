#include <tl_camera.h>

#include <cmath>

TLCam::TLCam(const glm::mat4& xform, const glm::ivec2& res, const float ver_fov) {
  camToWorld_xform = xform;
  resolution = res;
  vfov = ver_fov;
  float theta = (ver_fov * M_PI) / 180.0;  // degrees to radians

  // set Physical size of the image plane
  float ratio = (float)resolution[0] / resolution[1];
  float img_height = 2.0f * (tan(theta / 2.0f));
  float img_width = ratio * img_height;

  p_size[0] = img_width;
  p_size[1] = img_height;
}

Ray TLCam::generate_ray(const float& x, const float& y) const {
  Ray new_ray;
  /* lerp is slow
  new_ray.dir = glm::vec3(std::lerp(-p_size[0] / 2, p_size[0] / 2, x / resolution[0]),
                          std::lerp(-p_size[1] / 2, +p_size[1] / 2, y / resolution[1]), -1.0f);
   */
  float x_dir = (p_size[0] * (x / resolution[0])) - (p_size[0] / 2.0f);
  float y_dir = (p_size[1] * (y / resolution[1])) - (p_size[1] / 2.0f);

  new_ray.dir = glm::vec3(x_dir, y_dir, -1.0f);

  // Transforms from camera to world coords
  new_ray.xform_ray(camToWorld_xform);
  new_ray.dir = glm::normalize(new_ray.dir);
  return new_ray;
}

glm::mat4 camToWorld(const glm::vec3& lookFrom, const glm::vec3& lookAT, const glm::vec3& up) {
  glm::vec3 z_axis = glm::normalize(lookFrom - lookAT);
  glm::vec3 x_axis = glm::normalize(glm::cross(up, z_axis));
  glm::vec3 y_axis = glm::normalize(glm::cross(z_axis, x_axis));

  return glm::mat4(glm::vec4(x_axis, 0.0f), glm::vec4(y_axis, 0.0f), glm::vec4(z_axis, 0.0f),
                   glm::vec4(lookFrom, 1.0f));
}