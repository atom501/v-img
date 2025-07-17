#include <scene_loading/serialized_file.h>

#include <fstream>

void skip_to_idx(std::fstream &fs, const short version, const size_t shape_index) {
  // Go to the end of the file to read the total number of meshes in the .serialized file
  fs.seekg(-sizeof(uint32_t), fs.end);
  uint32_t count = 0;
  fs.read((char *)&count, sizeof(uint32_t));

  size_t offset = 0;
  if (version == 4) {  // v4 uses uint64
    fs.seekg(-sizeof(uint64_t) * (count - shape_index) - sizeof(uint32_t), fs.end);
    fs.read((char *)&offset, sizeof(size_t));
  } else {  // V3 uses uint32
    fs.seekg(-sizeof(uint32_t) * (count - shape_index + 1), fs.end);
    uint32_t upos = 0;
    fs.read((char *)&upos, sizeof(unsigned int));
    offset = upos;
  }
  fs.seekg(offset, fs.beg);
  // Skip the header
  fs.ignore(sizeof(short) * 2);
}

template <typename Precision> std::vector<glm::vec3> load_position(ZStream &zs, int num_vertices) {
  std::vector<glm::vec3> vertices(num_vertices);
  for (int i = 0; i < (int)num_vertices; i++) {
    Precision x, y, z;
    zs.read(&x, sizeof(Precision));
    zs.read(&y, sizeof(Precision));
    zs.read(&z, sizeof(Precision));
    vertices[i] = glm::vec3{x, y, z};
  }
  return vertices;
}

template <typename Precision> std::vector<glm::vec3> load_normal(ZStream &zs, int num_vertices) {
  std::vector<glm::vec3> normals(num_vertices);
  for (int i = 0; i < (int)normals.size(); i++) {
    Precision x, y, z;
    zs.read(&x, sizeof(Precision));
    zs.read(&y, sizeof(Precision));
    zs.read(&z, sizeof(Precision));
    normals[i] = glm::vec3{x, y, z};
  }
  return normals;
}

template <typename Precision> std::vector<glm::vec2> load_uv(ZStream &zs, int num_vertices) {
  std::vector<glm::vec2> uvs(num_vertices);
  for (int i = 0; i < (int)uvs.size(); i++) {
    Precision u, v;
    zs.read(&u, sizeof(Precision));
    zs.read(&v, sizeof(Precision));
    uvs[i] = glm::vec2{u, v};
  }
  return uvs;
}

// file format is described in mitsuba Serialized mesh loader section
Mesh read_serialized_file(const std::filesystem::path &filepath, int shape_index,
                          glm::mat4 to_world) {
  std::fstream file_stream(filepath.c_str(), std::fstream::in | std::fstream::binary);

  // ignore format identifier
  file_stream.ignore(sizeof(uint16_t));

  // version number
  uint16_t version = 0;
  file_stream.read((char *)&version, sizeof(uint16_t));

  // after version number stream is compressed by the DEFLATE algorithm
  // used encoding is that of the zlib library. I use miniz to inflate the stream

  // jump to the shape_index given in scene description
  if (shape_index > 0) {
    skip_to_idx(file_stream, version, shape_index);
  }

  // init zstream and start reading mesh
  ZStream zs(file_stream);

  uint32_t flags;
  zs.read((char *)&flags, sizeof(uint32_t));

  // mesh name
  std::string name;
  if (version == 4) {
    char c;
    while (true) {
      zs.read((char *)&c, sizeof(char));
      if (c == '\0') break;
      name.push_back(c);
    }
  }

  size_t vertex_count = 0;
  zs.read((char *)&vertex_count, sizeof(size_t));
  size_t triangle_count = 0;
  zs.read((char *)&triangle_count, sizeof(size_t));

  bool double_precision = flags & static_cast<uint32_t>(TriMeshFlags::DoublePrecision);

  std::vector<glm::vec3> vertices;
  if (double_precision) {
    vertices = load_position<double>(zs, vertex_count);
  } else {
    vertices = load_position<float>(zs, vertex_count);
  }

  for (auto &p : vertices) {
    glm::vec4 result = to_world * glm::vec4(p, 1);
    result /= result.w;
    p = result;
  }

  std::vector<glm::vec3> normals;
  if (flags & static_cast<uint32_t>(TriMeshFlags::HasNormals)) {
    if (double_precision) {
      normals = load_normal<double>(zs, vertex_count);
    } else {
      normals = load_normal<float>(zs, vertex_count);
    }

    const glm::mat4 normal_xform = glm::transpose(glm::inverse(to_world));
    for (auto &n : normals) {
      glm::vec4 result = normal_xform * glm::vec4(n, 0);
      n = result;
    }
  }

  std::vector<glm::vec2> texcoords;
  if (flags & static_cast<uint32_t>(TriMeshFlags::HasTexcoords)) {
    if (double_precision) {
      texcoords = load_uv<double>(zs, vertex_count);
    } else {
      texcoords = load_uv<float>(zs, vertex_count);
    }
  }

  // mitsuba only has one index array, instead of having a seperate one for each vertex attribute
  // TODO switch to mitsuba method later
  std::vector<uint32_t> tri_vertex(triangle_count * 3);
  std::vector<uint32_t> tri_normal(triangle_count * 3);
  std::vector<int> tri_uv(triangle_count * 3);

  for (size_t i = 0; i < triangle_count * 3; i++) {
    int index;
    zs.read(&index, sizeof(int));

    tri_vertex[i] = index;
    tri_normal[i] = index;
    tri_uv[i] = index;
  }

  return Mesh(vertices, tri_vertex, normals, tri_normal, texcoords, tri_uv);
}