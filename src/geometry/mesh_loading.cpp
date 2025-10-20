#include <fmt/core.h>
#include <geometry/mesh.h>
#define TINYOBJLOADER_IMPLEMENTATION
#include <tiny_obj_loader.h>

#include <filesystem>
#include <glm/matrix.hpp>
#include <unordered_map>

// struct ArrayHasher {
//   std::size_t operator()(const std::array<int, 3>& a) const {
//     std::size_t h = 0;

//     for (auto e : a) {
//       h ^= std::hash<int>{}(e) + 0x9e3779b9 + (h << 6) + (h >> 2);
//     }
//     return h;
//   }
// };

bool load_from_obj(std::string model_path_rel_file, std::vector<std::array<uint32_t, 3>>& indices,
                   std::vector<glm::vec3>& vertices, std::vector<glm::vec3>& normals,
                   std::vector<glm::vec2>& texcoords, const glm::mat4& transform) {
  tinyobj::attrib_t attrib;
  std::vector<tinyobj::shape_t> shapes;
  std::vector<tinyobj::material_t> materials;
  std::string warnings;
  std::string errors;

  if (tinyobj::LoadObj(&attrib, &shapes, &materials, &warnings, &errors,
                       model_path_rel_file.c_str())
      == false) {
    fmt::println("Tinyobj failed to load the mesh \n {}", errors);
    return false;
  }

  // only load vertices from obj for now
  // TODO add normals and texcoords loading

  // read vertices and normals, also transform them
  for (size_t i = 0; i < attrib.vertices.size(); i += 3) {
    auto vec = glm::vec3(attrib.vertices[i], attrib.vertices[i + 1], attrib.vertices[i + 2]);
    glm::vec4 result = transform * glm::vec4(vec, 1);
    result /= result.w;
    vertices.push_back(glm::vec3(result));
  }

  for (const auto& shape : shapes) {
    const std::vector<tinyobj::index_t>& indices_list = shape.mesh.indices;
    const std::vector<int>& material_ids = shape.mesh.material_ids;

    // for each triangle get the index in vector of vertices for a vertex
    for (size_t i = 0; i < material_ids.size(); i++) {
      // for ith triangle

      // get index for each vertex
      std::array<uint32_t, 3> idx = {static_cast<uint32_t>(indices_list[3 * i].vertex_index),
                                     static_cast<uint32_t>(indices_list[3 * i + 1].vertex_index),
                                     static_cast<uint32_t>(indices_list[3 * i + 2].vertex_index)};

      indices.push_back(idx);
    }
  }

  return true;
}
