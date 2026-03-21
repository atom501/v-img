#pragma once

#include <geometry/mesh.h>

#include <filesystem>

// flags used by mitsuba serialized file format
enum class TriMeshFlags : uint32_t {
  HasNormals = 0x0001,
  HasTexcoords = 0x0002,
  HasTangents = 0x0004,  // unused
  HasColors = 0x0008,
  FaceNormals = 0x0010,
  SinglePrecision = 0x1000,
  DoublePrecision = 0x2000
};

/*
 * Takes the path of the serialized file. The path is relative to mitsuba scene file.
 * Reads and returns a single mesh. Right now does not supports multiple meshes
 */
Mesh read_serialized_file(const std::filesystem::path& filepath, int shape_index,
                          glm::mat4 to_world);