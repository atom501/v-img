#include <geometry/mesh.h>
#include <geometry/triangle.h>
#include <material/diffuse_light.h>
#include <material/principled.h>
#include <scene_loading/gltf_loading.h>
#include <scene_loading/json_scene.h>
#include <texture.h>

#include <cstddef>
#include <fastgltf/core.hpp>
#include <fastgltf/tools.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/transform.hpp>
#include <variant>

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

// image and img_pair will change
static ImageTexture* make_texture(std::vector<glm::vec3>& image, const glm::ivec2& res,
                                  const std::optional<fastgltf::Sampler>& sampler_info,
                                  TextureType tex_type,
                                  std::vector<std::unique_ptr<Texture>>& texture_list,
                                  std::pair<uint32_t, TextureType>& img_pair, float scale) {
  // create the image texture
  TextureWrappingMode u_mode = TextureWrappingMode::Repeat;
  TextureWrappingMode v_mode = TextureWrappingMode::Repeat;

  std::vector<std::vector<glm::vec3>> no_mipmap;

  if (sampler_info) {
    u_mode = gltf_wrap_convert(sampler_info.value().wrapS);
    v_mode = gltf_wrap_convert(sampler_info.value().wrapT);
  }

  switch (tex_type) {
    case TextureType::Image:
      ImageTexture::convert_sRGB_to_linear(image);
      texture_list.push_back(std::make_unique<ImageTexture>(image, res.x, res.y, u_mode, v_mode));
      break;
    case TextureType::Normals:
      ImageTexture::convert_RGB_to_normal(image, scale);
      no_mipmap.push_back(image);
      texture_list.push_back(
          std::make_unique<ImageTexture>(no_mipmap, res.x, res.y, u_mode, v_mode));
      break;
    default:
      fmt::println("Error, not covered image texture type");
      return nullptr;
      break;
  }

  img_pair = std::make_pair(texture_list.size() - 1, tex_type);
  return dynamic_cast<ImageTexture*>(texture_list.back().get());
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

    fastgltf::iterateAccessor<fastgltf::math::fvec2>(
        asset, texCoordAccessor,
        [&](fastgltf::math::fvec2 uv) { texcoords.push_back(glm::vec2(uv.x(), uv.y())); });
  }

  return texcoords;
}

bool set_scene_from_gltf(const std::filesystem::path& path_file, integrator_data& integrator_data,
                         std::vector<std::unique_ptr<Surface>>& list_surfaces,
                         std::vector<std::unique_ptr<Material>>& list_materials,
                         std::vector<Emitter*>& list_lights,
                         std::vector<std::unique_ptr<Mesh>>& list_meshes,
                         std::vector<std::unique_ptr<Texture>>& texture_list,
                         const nlohmann::json& extra_settings) {
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
  std::vector<std::vector<glm::vec3>> image_list;
  std::vector<glm::ivec2> image_list_res;
  for (auto& img : asset->images) {
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
        img.data);
    std::vector<glm::vec3> image_255(width * height);

    for (size_t i = 0; i < image_255.size(); i++) {
      for (size_t c = 0; c < 3; c++) image_255[i][c] = image_loaded[i * 3 + c];
    }

    stbi_image_free(image_loaded);
    image_list.push_back(image_255);
    image_list_res.push_back(glm::ivec2(width, height));
  }

  // load material
  std::vector<std::pair<uint32_t, TextureType>> img_list_idx(asset->images.size(),
                                                             std::make_pair(0, TextureType::None));

  for (auto& mat : asset->materials) {
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
      Texture* img_tex = nullptr;
      if (baseColorTexture.has_value()) {
        // add image texture
        auto& texture = asset->textures[baseColorTexture->textureIndex];
        if (!texture.imageIndex.has_value()) {
          fmt::println("Image texture has no image index");
          return false;
        }

        size_t image_idx = texture.imageIndex.value();
        auto tex_type = img_list_idx[image_idx].second;

        if (tex_type == TextureType::Image) {
          img_tex = texture_list[img_list_idx[image_idx].first].get();
        } else if (tex_type == TextureType::None) {
          // create the image texture

          std::optional<fastgltf::Sampler> sampler_info = std::nullopt;
          if (texture.samplerIndex) {
            sampler_info = asset->samplers[texture.samplerIndex.value()];
          }

          img_tex = make_texture(image_list[image_idx], image_list_res[image_idx], sampler_info,
                                 TextureType::Image, texture_list, img_list_idx[image_idx], 0.f);
        } else {
          fmt::println("Texture being loaded is already of type {} not a Image", tex_type);
          return false;
        }

      } else {
        // add constant texture
        texture_list.push_back(std::make_unique<ConstColor>(
            glm::vec3(mat.pbrData.baseColorFactor.x(), mat.pbrData.baseColorFactor.y(),
                      mat.pbrData.baseColorFactor.z())));

        img_tex = texture_list.back().get();
      }

      ImageTexture* normal_tex = nullptr;

      if (mat.normalTexture.has_value()) {
        // load normal map if present

        const auto& normal_tex_info = mat.normalTexture.value();
        auto& normal_texture = asset->textures[normal_tex_info.textureIndex];

        if (!normal_texture.imageIndex.has_value()) {
          fmt::println("Normal texture has no image index");
          return false;
        }
        size_t image_idx = normal_texture.imageIndex.value();
        auto tex_type = img_list_idx[image_idx].second;

        if (tex_type == TextureType::Normals) {
          normal_tex
              = dynamic_cast<ImageTexture*>(texture_list[img_list_idx[image_idx].first].get());
        } else if (tex_type == TextureType::None) {
          // make a normal texture

          std::optional<fastgltf::Sampler> sampler_info = std::nullopt;
          if (normal_texture.samplerIndex) {
            sampler_info = asset->samplers[normal_texture.samplerIndex.value()];
          }

          normal_tex = make_texture(image_list[image_idx], image_list_res[image_idx], sampler_info,
                                    TextureType::Normals, texture_list, img_list_idx[image_idx],
                                    normal_tex_info.scale);
        } else {
          fmt::println("Texture being loaded is already of type {} not a Normal", tex_type);
        }
      }

      list_materials.push_back(std::make_unique<Principled>(
          img_tex, specular_transmission, metallic, subsurface, specular, roughness, spec_tint,
          anisotropic, sheen, sheen_tint, clearcoat, clearcoat_gloss, eta, normal_tex));
    } else {
      glm::vec3 emit
          = glm::vec3(mat.emissiveFactor[0], mat.emissiveFactor[1], mat.emissiveFactor[2]);
      emit *= mat.emissiveStrength;

      list_materials.push_back(std::make_unique<DiffuseLight>(emit));
    }
  }

  // load meshes and transform them

  // loop over nodes in hierarchy check if meshindex, load transform, load mesh. (set default
  // material for now)
  for (size_t scene_idx = 0; scene_idx < asset->scenes.size(); scene_idx++) {
    fastgltf::iterateSceneNodes(
        asset.get(), scene_idx, fastgltf::math::fmat4x4(),
        [&](fastgltf::Node& node, fastgltf::math::fmat4x4 node_matrix) {
          if (node.meshIndex.has_value()) {
            // data for mesh object
            std::vector<glm::vec3> vertices;
            std::vector<glm::vec3> normals;
            std::vector<glm::vec2> texcoords;
            std::vector<glm::vec2> normal_coords;

            std::vector<std::array<uint32_t, 3>> indices;

            fastgltf::Mesh& mesh = asset->meshes[node.meshIndex.value()];
            glm::mat4 mesh_to_world;

            for (size_t x = 0; x < 4; x++) {
              for (size_t y = 0; y < 4; y++) {
                mesh_to_world[x][y] = node_matrix[x][y];
              }
            }

            for (auto it = mesh.primitives.begin(); it != mesh.primitives.end(); ++it) {
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

              fastgltf::iterateAccessor<fastgltf::math::fvec3>(
                  asset.get(), positionAccessor, [&](fastgltf::math::fvec3 pos) {
                    auto vec = glm::vec3(pos.x(), pos.y(), pos.z());
                    glm::vec4 result = mesh_to_world * glm::vec4(vec, 1);
                    result /= result.w;
                    vertices.push_back(result);
                  });

              // load normals
              if (const auto* normal_att = it->findAttribute("NORMAL");
                  normal_att != it->attributes.end()) {
                auto& normalAccessor = asset->accessors[normal_att->accessorIndex];
                if (normalAccessor.bufferViewIndex.has_value()) {
                  const glm::mat4 normal_xform = glm::transpose(glm::inverse(mesh_to_world));

                  fastgltf::iterateAccessor<fastgltf::math::fvec3>(
                      asset.get(), normalAccessor, [&](fastgltf::math::fvec3 normal) {
                        auto norm = glm::vec3(normal.x(), normal.y(), normal.z());
                        glm::vec4 result = normal_xform * glm::vec4(norm, 0);
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

              // add to list of surfaces
              for (size_t i = 0; i < tri_vertex.size() / 3; i++) {
                Triangle t = Triangle(list_meshes[list_meshes.size() - 1].get(), i);
                list_surfaces.push_back(std::make_unique<Triangle>(t));
              }

              // add to list of lights if needed
              size_t rev_count_index = list_surfaces.size() - 1;

              if (mat_ptr->is_emissive()) {
                for (size_t i = 0; i < tri_vertex.size() / 3; i++, rev_count_index--) {
                  Triangle* s_ptr = static_cast<Triangle*>(list_surfaces[rev_count_index].get());
                  list_lights.push_back(s_ptr);
                }
              }
            }
          }
        });
  }

  return true;
}
