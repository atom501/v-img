#include <comptime_settings.h>
#include <geometry/mesh.h>
#include <geometry/triangle.h>
#include <material/diffuse_light.h>
#include <material/principled.h>
#include <omp.h>
#include <scene_loading/gltf_loading.h>
#include <scene_loading/json_scene.h>
#include <texture/texture_RGB.h>

#include <atomic>
#include <cstddef>
#include <fastgltf/core.hpp>
#include <fastgltf/tools.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/transform.hpp>
#include <variant>

#include "fastgltf/glm_element_traits.hpp"
#include "fastgltf/tools.hpp"
#include "fastgltf/types.hpp"
#include "stb_image.h"

static inline glm::mat4 get_mat_transform(
    std::variant<fastgltf::TRS, fastgltf::math::fmat4x4>& gltf_xform) {
  glm::mat4 glm_xform = glm::mat4(1.0f);
  std::visit(
      fastgltf::visitor{
          [&](fastgltf::TRS& TRS_xform) {
            // make matrix from TRS
            glm_xform = glm::scale(glm::vec3(TRS_xform.scale.x(), TRS_xform.scale.y(),
                                             TRS_xform.scale.z()))
                        * glm_xform;
            glm_xform = glm::toMat4(glm::quat(TRS_xform.rotation.w(), TRS_xform.rotation.x(),
                                              TRS_xform.rotation.y(), TRS_xform.rotation.z()))
                        * glm_xform;
            glm_xform
                = glm::translate(glm::vec3(TRS_xform.translation.x(), TRS_xform.translation.y(),
                                           TRS_xform.translation.z()))
                  * glm_xform;
          },
          [&](fastgltf::math::fmat4x4& mat_xform) {
            // copy matrix
            for (size_t x = 0; x < 4; x++) {
              for (size_t y = 0; y < 4; y++) {
                glm_xform[x][y] = mat_xform[x][y];
              }
            }
          },
      },
      gltf_xform);

  return glm_xform;
}

static inline void set_vals(
    fastgltf::Optional<size_t> sampler_idx, fastgltf::Optional<size_t> image_idx,
    const fastgltf::Asset& asset, TextureType type,
    std::vector<std::tuple<TextureType, TextureWrappingMode, TextureWrappingMode, float>>&
        img_type_idx,
    float scale) {
  if (!image_idx.has_value()) {
    fmt::println("Image texture has no image index");
  }

  if (sampler_idx) {
    auto sampler_info = asset.samplers[sampler_idx.value()];

    get<1>(img_type_idx[image_idx.value()]) = gltf_wrap_convert(sampler_info.wrapS);
    get<2>(img_type_idx[image_idx.value()]) = gltf_wrap_convert(sampler_info.wrapT);
  }

  get<0>(img_type_idx[image_idx.value()]) = type;
  get<3>(img_type_idx[image_idx.value()]) = scale;
}

static void set_image_type_list(
    std::vector<std::tuple<TextureType, TextureWrappingMode, TextureWrappingMode, float>>&
        img_type_idx,
    const fastgltf::Asset& asset, bool& unique_imgs) {
  size_t total_images = img_type_idx.size();
  size_t img_found = 0;

  if (total_images == 0) {
    return;
  }

  for (const auto& mat : asset.materials) {
    auto& baseColorTexture = mat.pbrData.baseColorTexture;
    if (baseColorTexture.has_value()) {
      auto& texture = asset.textures[baseColorTexture->textureIndex];

      set_vals(texture.samplerIndex, texture.imageIndex, asset, TextureType::Image, img_type_idx,
               0.f);

      img_found++;
    }

    if (mat.normalTexture.has_value()) {
      const auto& normal_tex_info = mat.normalTexture.value();
      auto& normal_texture = asset.textures[normal_tex_info.textureIndex];

      set_vals(normal_texture.samplerIndex, normal_texture.imageIndex, asset, TextureType::Normals,
               img_type_idx, normal_tex_info.scale);

      img_found++;
    }
  }

  if (img_found > total_images) {
    fmt::println("Error one image is being used for as a texture and a normalmap");
    unique_imgs = false;
  }
}

// image and img_pair will change
static void make_texture(unsigned char* char_arr, const glm::ivec2& res, TextureWrappingMode u_mode,
                         TextureWrappingMode v_mode, TextureType tex_type,
                         std::vector<std::unique_ptr<TextureRGB>>& texture_list, float scale,
                         size_t tex_write_idx) {
  std::vector<glm::vec3> image(res.x * res.y);

  for (size_t i = 0; i < image.size(); i++) {
    image[i].x = char_arr[i * 3];
    image[i].y = char_arr[i * 3 + 1];
    image[i].z = char_arr[i * 3 + 2];
  }

  // create the image texture
  switch (tex_type) {
    case TextureType::Image: {
      ImageTexture::convert_sRGB_to_linear(image);

      std::unique_ptr<ImageTexture> img_tex
          = std::make_unique<ImageTexture>(std::move(image), res.x, res.y, u_mode, v_mode);

      texture_list[tex_write_idx] = std::move(img_tex);

      break;
    }
    case TextureType::Normals: {
      std::vector<std::vector<glm::vec3>> no_mipmap;

      ImageTexture::convert_RGB_to_normal(image, scale);
      no_mipmap.push_back(image);

      std::unique_ptr<ImageTexture> n_tex
          = std::make_unique<ImageTexture>(std::move(no_mipmap), res.x, res.y, u_mode, v_mode);

      texture_list[tex_write_idx] = std::move(n_tex);

      break;
    }
    default:
      fmt::println("Error, not covered image texture type");
      break;
  }
}

std::vector<glm::vec2> get_texcoords(size_t texcoord_idx, const fastgltf::Primitive& primitive,
                                     const fastgltf::Asset& asset) {
  auto texcoordAttribute = std::string("TEXCOORD_") + std::to_string(texcoord_idx);
  std::vector<glm::vec2> texcoords;

  if (const auto* texcoord_acc = primitive.findAttribute(texcoordAttribute);
      texcoord_acc != primitive.attributes.end()) {
    // Tex coord
    auto& texCoordAccessor = asset.accessors[texcoord_acc->accessorIndex];
    if (!texCoordAccessor.bufferViewIndex.has_value()) {
      fmt::println(
          "texCoordAccessor.bufferViewIndex has no value. skipping "
          "mesh.primitive");
    }

    fastgltf::iterateAccessor<glm::vec2>(asset, texCoordAccessor,
                                         [&](glm::vec2 uv) { texcoords.push_back(uv); });
  }

  return texcoords;
}

bool set_scene_from_gltf(const std::filesystem::path& path_file, integrator_data& integrator_data,
                         std::vector<std::unique_ptr<Surface>>& list_surfaces,
                         std::vector<std::unique_ptr<Material>>& list_materials,
                         std::vector<Emitter*>& list_lights,
                         std::vector<std::unique_ptr<Mesh>>& list_meshes,
                         std::vector<std::unique_ptr<TextureRGB>>& texture_list,
                         const nlohmann::json& extra_settings) {
  omp_set_max_active_levels(2);
  bool unique_imgs = true;

  fastgltf::Expected<fastgltf::GltfDataBuffer> gltfFile
      = fastgltf::GltfDataBuffer::FromPath(path_file);

  if (!bool(gltfFile)) {
    fmt::println("Failed to open glTF file: {}", fastgltf::getErrorMessage(gltfFile.error()));
    return false;
  }

  static constexpr auto supportedExtensions = fastgltf::Extensions::KHR_materials_emissive_strength
                                              | fastgltf::Extensions::KHR_materials_specular
                                              | fastgltf::Extensions::KHR_materials_ior
                                              | fastgltf::Extensions::KHR_materials_transmission
                                              | fastgltf::Extensions::KHR_materials_clearcoat
                                              | fastgltf::Extensions::KHR_materials_transmission
                                              | fastgltf::Extensions::KHR_materials_sheen
                                              | fastgltf::Extensions::KHR_materials_anisotropy;

  fastgltf::Parser parser(supportedExtensions);
  fastgltf::Expected<fastgltf::Asset> asset
      = parser.loadGltf(gltfFile.get(), path_file.parent_path());

  if (!bool(asset)) {
    fmt::println("Failed to open glTF: {}", fastgltf::getErrorMessage(asset.error()));
    return false;
  }

  // only load the first camera
  if (asset->cameras.size() == 0) {
    fmt::println("No camera in the scene");
    return false;
  }
  auto& scene_camera = asset->cameras[0];
  glm::mat4 camToWorld = glm::mat4(1.0f);
  float vfov_rad = 0.f;
  float aspect_ratio = 1.f;

  if (std::holds_alternative<fastgltf::Camera::Perspective>(scene_camera.camera)) {
    for (auto& node : asset->nodes) {
      if (node.cameraIndex.has_value()) {
        std::variant<fastgltf::TRS, fastgltf::math::fmat4x4>& cam_xform = node.transform;
        camToWorld = get_mat_transform(cam_xform);

        // stop after finding the first camera
        break;
      }
    }

    auto p_cam = std::get<fastgltf::Camera::Perspective>(scene_camera.camera);

    vfov_rad = p_cam.yfov;
    aspect_ratio = p_cam.aspectRatio.value_or(1.f);

  } else {
    fmt::println("Failed to open glTF: Orthographic camera is not supported");
  }

  // load the extra settings from the companion json file
  integrator_data.samples = extra_settings.value("spp", 32);
  integrator_data.depth = extra_settings.value("depth", 64);

  if (extra_settings.contains("integrator")) {
    std::string integrator_string = extra_settings["integrator"];

    if (integrator_string == "s_normal") {
      integrator_data.func = integrator_func::s_normal;
      fmt::println("Shading normal integrator set");
    } else if (integrator_string == "g_normal") {
      integrator_data.func = integrator_func::g_normal;
      fmt::println("Shading normal integrator set");
    } else if (integrator_string == "material") {
      integrator_data.func = integrator_func::material;
      fmt::println("Material integrator set");
    } else if (integrator_string == "mis") {
      integrator_data.func = integrator_func::mis;
      fmt::println("MIS integrator set");
    } else {
      integrator_data.func = integrator_func::s_normal;
      fmt::println("Integrator {} is not defined. Set to shading normal", integrator_string);
    }
  } else {
    integrator_data.func = integrator_func::s_normal;
  }

  std::thread background_loader = std::thread([&]() {
    std::chrono::steady_clock::time_point begin_time;

    if constexpr (CompileConsts::profile_gltf) {
      begin_time = std::chrono::steady_clock::now();
    }

    integrator_data.background = std::make_unique<ConstBackground>(ConstBackground(glm::vec3(0)));

    if (extra_settings.contains("background")) {
      if (extra_settings["background"].is_array()) {
        const auto& val = extra_settings["background"];
        integrator_data.background = std::make_unique<ConstBackground>(
            ConstBackground(extra_settings["background"].template get<glm::vec3>()));
        list_lights.push_back(static_cast<ConstBackground*>(integrator_data.background.get()));
      } else {
        std::filesystem::path env_filepath = extra_settings["background"];
        // read exr file
        std::string env_type = env_filepath.extension().generic_string();
        if (env_type == ".exr") {
          std::filesystem::path scene_filename = path_file;
          const auto env_path_rel_file = scene_filename.remove_filename() / env_filepath;

          ImageTexture image = load_imagetexture(env_path_rel_file);

          float radiance_scale = extra_settings.value("radiance_scale", 1.f);

          // set env map
          integrator_data.background = std::make_unique<EnvMap>(
              EnvMap(image, glm::mat4(1.0f), glm::mat4(1.0f), radiance_scale));

          list_lights.push_back(static_cast<EnvMap*>(integrator_data.background.get()));

        } else {
          fmt::println("env map file type {} is not supported. Settings background to black",
                       env_type);
        }
      }
    }

    if constexpr (CompileConsts::profile_gltf) {
      auto end_time = std::chrono::steady_clock::now();
      print_time_taken(begin_time, end_time, "loading gltf background");
    }
  });

  std::vector<std::tuple<TextureType, TextureWrappingMode, TextureWrappingMode, float>>
      img_type_list(asset->images.size(),
                    std::make_tuple(TextureType::None, TextureWrappingMode::Repeat,
                                    TextureWrappingMode::Repeat, 0.f));

  std::thread set_img_types = std::thread(set_image_type_list, std::ref(img_type_list),
                                          std::ref(asset.get()), std::ref(unique_imgs));

  // use the passed in vertical resolution and aspect ratio
  integrator_data.resolution.y = extra_settings.value("yres", 768);
  integrator_data.resolution.x = std::ceil(integrator_data.resolution.y * aspect_ratio);

  float focal_dist = 1.f;
  float aperture_radius = 0.f;

  if (extra_settings.contains("camera")
      && (integrator_data.func == integrator_func::mis
          || integrator_data.func == integrator_func::material)) {
    focal_dist = extra_settings["camera"].value("fdist", 1.f);
    aperture_radius = extra_settings["camera"].value("aperture_radius", 0.f);
  }

  integrator_data.camera
      = TLCam(camToWorld, integrator_data.resolution, vfov_rad * (180.f / fastgltf::math::pi),
              aperture_radius, focal_dist);

  // load images as 24 bit. each channel having 8 bits. To be transformed when making materials
  std::vector<unsigned char*> image_list(asset->images.size());
  std::vector<glm::ivec2> image_list_res(asset->images.size());

  std::chrono::steady_clock::time_point begin_time;

  if constexpr (CompileConsts::profile_gltf) {
    begin_time = std::chrono::steady_clock::now();
  }

#pragma omp parallel for
  for (int img_idx = 0; img_idx < asset->images.size(); img_idx++) {
    unsigned char* image_loaded;
    int width, height, nrChannels;

    std::visit(
        fastgltf::visitor{
            [](auto& arg) {},
            [&](fastgltf::sources::URI& filePath) {
              assert(filePath.fileByteOffset == 0);
              assert(filePath.uri.isLocalPath());

              const std::string path(filePath.uri.path().begin(), filePath.uri.path().end());
              image_loaded = stbi_load(path.c_str(), &width, &height, &nrChannels, STBI_rgb);
            },
            [&](fastgltf::sources::Array& vector) {
              image_loaded = stbi_load_from_memory(
                  reinterpret_cast<const stbi_uc*>(vector.bytes.data()),
                  static_cast<int>(vector.bytes.size()), &width, &height, &nrChannels, STBI_rgb);
            },
            [&](fastgltf::sources::BufferView& view) {
              auto& bufferView = asset->bufferViews[view.bufferViewIndex];
              auto& buffer = asset->buffers[bufferView.bufferIndex];

              std::visit(fastgltf::visitor{[](auto& arg) {},
                                           [&](fastgltf::sources::Array& vector) {
                                             image_loaded = stbi_load_from_memory(
                                                 reinterpret_cast<const stbi_uc*>(
                                                     vector.bytes.data() + bufferView.byteOffset),
                                                 static_cast<int>(bufferView.byteLength), &width,
                                                 &height, &nrChannels, STBI_rgb);
                                           }},
                         buffer.data);
            },
        },
        asset->images[img_idx].data);

    image_list[img_idx] = image_loaded;
    image_list_res[img_idx] = glm::ivec2(width, height);
  }

  if constexpr (CompileConsts::profile_gltf) {
    auto end_time = std::chrono::steady_clock::now();
    print_time_taken(begin_time, end_time, "loading gltf images");
  }

  /*
   * set maximum possible number of textures to avoid lock and push_back. (assumes one image is not
   * used as normal map and also a pbr color)
   */
  texture_list = std::vector<std::unique_ptr<TextureRGB>>(asset->materials.size()
                                                          + asset->images.size() + 1);

  list_materials = std::vector<std::unique_ptr<Material>>(asset->materials.size());
  set_img_types.join();

  if (!unique_imgs) {
    return false;
  }

  // set textures
  if constexpr (CompileConsts::profile_gltf) {
    begin_time = std::chrono::steady_clock::now();
  }

#pragma omp parallel for
  for (size_t t_idx = 0; t_idx < img_type_list.size(); t_idx++) {
    auto [tex_type, u_wrap, v_wrap, scale] = img_type_list[t_idx];
    switch (tex_type) {
      case TextureType::Image: {
        make_texture(image_list[t_idx], image_list_res[t_idx], u_wrap, v_wrap, TextureType::Image,
                     texture_list, 0.f, t_idx);
        break;
      }
      case TextureType::Normals: {
        make_texture(image_list[t_idx], image_list_res[t_idx], u_wrap, v_wrap, TextureType::Normals,
                     texture_list, scale, t_idx);
        break;
      }
      default:
        break;
    }
  }

  if constexpr (CompileConsts::profile_gltf) {
    auto end_time = std::chrono::steady_clock::now();
    print_time_taken(begin_time, end_time, "creating textures");
  }

  // idx where new texture is to be written. start adding consts after imgs
  std::atomic<unsigned int> tex_write_idx = img_type_list.size();

  if constexpr (CompileConsts::profile_gltf) {
    begin_time = std::chrono::steady_clock::now();
  }

// load material
#pragma omp parallel for
  for (int mat_idx = 0; mat_idx < asset->materials.size(); mat_idx++) {
    const auto& mat = asset->materials[mat_idx];

    // check if material is emissive
    if (mat.emissiveFactor[0] == 0.f && mat.emissiveFactor[1] == 0.f
        && mat.emissiveFactor[2] == 0.f) {
      float metallic = mat.pbrData.metallicFactor;
      float roughness = mat.pbrData.roughnessFactor;
      glm::vec3 base_color
          = glm::vec3(mat.pbrData.baseColorFactor[0], mat.pbrData.baseColorFactor[1],
                      mat.pbrData.baseColorFactor[2]);

      float anisotropic = 0.f;
      if (mat.anisotropy) {
        anisotropic = mat.anisotropy->anisotropyStrength;
      }

      float sheen = 0.f;
      float sheen_tint = 0.5f;
      if (mat.sheen) {
        sheen = mat.sheen->sheenRoughnessFactor;

        // not sure
        sheen_tint = mat.sheen->sheenColorFactor[0];
      }

      float clearcoat = 0.f;
      float clearcoat_gloss = 1.f;
      if (mat.clearcoat) {
        clearcoat = mat.clearcoat->clearcoatFactor;
        clearcoat_gloss = 1.f - mat.clearcoat->clearcoatRoughnessFactor;
      }

      float eta = mat.ior;

      float specular = 0.5f;
      float spec_tint = 0.f;
      if (mat.specular) {
        specular = mat.specular->specularFactor;
        spec_tint = mat.specular->specularColorFactor[0];
      }

      float specular_transmission = 0.f;
      if (mat.transmission) {
        specular_transmission = mat.transmission->transmissionFactor;
      }

      static constexpr float subsurface = 0.f;

      // get texture for material
      auto& baseColorTexture = mat.pbrData.baseColorTexture;
      TextureRGB* img_tex = nullptr;
      if (baseColorTexture.has_value()) {
        // add image texture
        auto& texture = asset->textures[baseColorTexture->textureIndex];
        if (!texture.imageIndex.has_value()) {
          fmt::println("Image texture has no image index");
        }

        size_t image_idx = texture.imageIndex.value();

        TextureType tex_type;
        std::tie(tex_type, std::ignore, std::ignore, std::ignore) = img_type_list[image_idx];

        if (tex_type == TextureType::Image) {
          img_tex = texture_list[image_idx].get();
        } else {
          fmt::println("TextureRGB being loaded is already of type {} not a Image", tex_type);
        }

      } else {
        // add constant texture
        auto idx = tex_write_idx.fetch_add(1);

        texture_list[idx] = std::make_unique<ConstColor>(
            glm::vec3(mat.pbrData.baseColorFactor.x(), mat.pbrData.baseColorFactor.y(),
                      mat.pbrData.baseColorFactor.z()));
        img_tex = texture_list[idx].get();
      }

      ImageTexture* normal_tex = nullptr;

      if (mat.normalTexture.has_value()) {
        // load normal map if present

        const auto& normal_tex_info = mat.normalTexture.value();
        auto& normal_texture = asset->textures[normal_tex_info.textureIndex];

        if (!normal_texture.imageIndex.has_value()) {
          fmt::println("Normal texture has no image index");
        }
        size_t image_idx = normal_texture.imageIndex.value();

        TextureType tex_type;
        std::tie(tex_type, std::ignore, std::ignore, std::ignore) = img_type_list[image_idx];

        if (tex_type == TextureType::Normals) {
          normal_tex = dynamic_cast<ImageTexture*>(texture_list[image_idx].get());
        } else {
          fmt::println("TextureRGB being loaded is already of type {} not a Normal", tex_type);
        }
      }

      list_materials[mat_idx] = std::make_unique<Principled>(
          img_tex, specular_transmission, metallic, subsurface, specular, roughness, spec_tint,
          anisotropic, sheen, sheen_tint, clearcoat, clearcoat_gloss, eta, normal_tex);
    } else {
      glm::vec3 emit
          = glm::vec3(mat.emissiveFactor[0], mat.emissiveFactor[1], mat.emissiveFactor[2]);
      emit *= mat.emissiveStrength;

      list_materials[mat_idx] = std::make_unique<DiffuseLight>(emit);
    }
  }

  if constexpr (CompileConsts::profile_gltf) {
    auto end_time = std::chrono::steady_clock::now();
    print_time_taken(begin_time, end_time, "loading gltf materials");
  }

  background_loader.join();
  for (auto& ptr : image_list) {
    stbi_image_free(ptr);
  }

  texture_list.resize(tex_write_idx.load());

  // load meshes and transform them

  if constexpr (CompileConsts::profile_gltf) {
    begin_time = std::chrono::steady_clock::now();
  }

  // loop over nodes in hierarchy check if meshindex, load transform, load mesh. (set default
  // material for now)
  for (size_t scene_idx = 0; scene_idx < asset->scenes.size(); scene_idx++) {
    fastgltf::iterateSceneNodes(
        asset.get(), scene_idx, fastgltf::math::fmat4x4(),
        [&](fastgltf::Node& node, fastgltf::math::fmat4x4 node_matrix) {
          if (node.meshIndex.has_value()) {
            fastgltf::Mesh& mesh = asset->meshes[node.meshIndex.value()];
            glm::mat4 mesh_to_world;

            for (size_t x = 0; x < 4; x++) {
              for (size_t y = 0; y < 4; y++) {
                mesh_to_world[x][y] = node_matrix[x][y];
              }
            }

            for (auto it = mesh.primitives.begin(); it != mesh.primitives.end(); ++it) {
              // data for mesh object
              std::vector<glm::vec3> vertices;
              std::vector<glm::vec3> normals;
              std::vector<glm::vec2> texcoords;
              std::vector<glm::vec2> normal_coords;

              std::vector<std::array<uint32_t, 3>> indices;

              auto* positionIt = it->findAttribute("POSITION");
              assert(positionIt != it->attributes.end());  // A mesh primitive is required to hold
                                                           // the POSITION attribute.
              assert(it->indicesAccessor.has_value());     // We specify GenerateMeshIndices, so we
                                                           // should always have indices

              // Get the output primitive
              auto index = std::distance(mesh.primitives.begin(), it);

              // loading vertex positions
              auto& positionAccessor = asset->accessors[positionIt->accessorIndex];
              if (!positionAccessor.bufferViewIndex.has_value()) {
                fmt::println(
                    "positionAccessor.bufferViewIndex has no value. skipping mesh.primitive");
                continue;
              }

              fastgltf::iterateAccessor<glm::vec3>(
                  asset.get(), positionAccessor, [&](glm::vec3 pos) {
                    glm::vec4 result = mesh_to_world * glm::vec4(pos, 1);
                    result /= result.w;
                    vertices.push_back(result);
                  });

              // load normals
              if (const auto* normal_att = it->findAttribute("NORMAL");
                  normal_att != it->attributes.end()) {
                auto& normalAccessor = asset->accessors[normal_att->accessorIndex];
                if (normalAccessor.bufferViewIndex.has_value()) {
                  const glm::mat4 normal_xform = glm::transpose(glm::inverse(mesh_to_world));

                  fastgltf::iterateAccessor<glm::vec3>(
                      asset.get(), normalAccessor, [&](glm::vec3 normal) {
                        glm::vec4 result = normal_xform * glm::vec4(normal, 0);
                        normals.push_back(glm::normalize(glm::vec3(result)));
                      });
                }
              }

              // load texcoords
              if (it->materialIndex.has_value()) {
                auto& material = asset->materials[it->materialIndex.value()];

                auto& baseColorTexture = material.pbrData.baseColorTexture;

                if (baseColorTexture.has_value()) {
                  auto& texture = asset->textures[baseColorTexture->textureIndex];

                  if (texture.imageIndex.has_value()) {
                    // texture transformation not supported
                    size_t baseColorTexcoordIndex = baseColorTexture->texCoordIndex;
                    texcoords = get_texcoords(baseColorTexcoordIndex, *it, asset.get());
                  }
                }

                if (material.normalTexture.has_value()) {
                  auto& normal_texture = asset->textures[material.normalTexture->textureIndex];

                  if (normal_texture.imageIndex.has_value()) {
                    size_t normal_tex_coords_idx = material.normalTexture->texCoordIndex;
                    normal_coords = get_texcoords(normal_tex_coords_idx, *it, asset.get());
                  }
                }
              }

              // Create the index buffer and copy the indices into it.

              std::vector<uint32_t> tri_vertex;

              auto& indexAccessor = asset->accessors[it->indicesAccessor.value()];
              if (!indexAccessor.bufferViewIndex.has_value()) {
                fmt::println(
                    "triangle indexAccessor.bufferViewIndex has no value. skipping mesh.primitive");
                continue;
              }

              fastgltf::iterateAccessor<std::uint32_t>(
                  asset.get(), indexAccessor, [&](std::uint32_t i) { tri_vertex.push_back(i); });

              for (size_t i = 0; i < tri_vertex.size(); i += 3) {
                std::array<uint32_t, 3> tri_indexes
                    = {tri_vertex[i], tri_vertex[i + 1], tri_vertex[i + 2]};

                indices.push_back(tri_indexes);
              }

              // load material
              Material* mat_ptr = nullptr;
              if (it->materialIndex.has_value()) {
                size_t mat_id = it->materialIndex.value();
                mat_ptr = list_materials[mat_id].get();
              } else {
                fmt::println("can't load mat. skipping mesh.primitive");
                continue;
              }

              auto m = Mesh(indices, vertices, normals, texcoords, normal_coords, mat_ptr);
              list_meshes.push_back(std::make_unique<Mesh>(m));

              add_tri_list_to_scene(list_surfaces, list_meshes.back().get(), list_lights);
            }
          }
        });
  }

  if constexpr (CompileConsts::profile_gltf) {
    auto end_time = std::chrono::steady_clock::now();
    print_time_taken(begin_time, end_time, "loading gltf meshes");
  }

  return true;
}
