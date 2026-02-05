#include <fmt/core.h>
#include <geometry/mesh.h>
#define TINYOBJLOADER_IMPLEMENTATION
#include <geometry/triangle.h>
#include <tiny_obj_loader.h>

#include <glm/matrix.hpp>

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

Mesh create_quad_mesh(Material* mat_ptr, const glm::mat4& xform) {
  std::vector<glm::vec3> vertices
      = {glm::vec3{-1, -1, 0}, glm::vec3{-1, 1, 0}, glm::vec3{1, 1, 0}, glm::vec3{1, -1, 0}};

  for (auto& vec : vertices) {
    glm::vec4 result = xform * glm::vec4(vec, 1);
    result /= result.w;
    vec = result;
  }

  std::vector<glm::vec3> normals;

  std::vector<glm::vec2> texcoords
      = {glm::vec2{0, 0}, glm::vec2{0, 1}, glm::vec2{1, 1}, glm::vec2{1, 0}};

  std::vector<std::array<uint32_t, 3>> indices = {{0, 2, 1}, {2, 0, 3}};

  return Mesh(indices, vertices, normals, texcoords, mat_ptr);
}

void add_tri_list_to_scene(const Mesh& mesh, std::vector<std::unique_ptr<Surface>>& list_surfaces,
                           std::vector<std::unique_ptr<Mesh>>& list_meshes,
                           std::vector<Emitter*>& list_lights) {
  size_t num_tri = mesh.indices.size();

  // add to list of surfaces
  for (size_t i = 0; i < num_tri; i++) {
    list_surfaces.push_back(
        std::make_unique<Triangle>(list_meshes[list_meshes.size() - 1].get(), i));
  }

  // add to list of lights if needed
  if (mesh.mat->is_emissive()) {
    size_t rev_count_index = list_surfaces.size() - 1;

    for (size_t i = rev_count_index; i > (rev_count_index - num_tri); i--) {
      list_lights.push_back(static_cast<Triangle*>(list_surfaces[i].get()));
    }
  }
}