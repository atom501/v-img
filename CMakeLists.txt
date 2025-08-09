cmake_minimum_required(VERSION 3.6...3.22)

# ---- Project ----

# Note: update this to your new project's name and version
project(v-img)

# --- Import tools ----

include(cmake/tools.cmake)

# ---- Dependencies ----

include(cmake/CPM.cmake)

# ---- Include guards ----

if(PROJECT_SOURCE_DIR STREQUAL PROJECT_BINARY_DIR)
  message(
    FATAL_ERROR
      "In-source builds not allowed. Please make a new directory (called a build directory) and run CMake from there."
  )
endif()

# ---- Add dependencies via CPM ----
# see https://github.com/TheLartians/CPM.cmake for more info

include(cmake/CPM.cmake)

# PackageProject.cmake will be used to make our target installable
CPMAddPackage("gh:TheLartians/PackageProject.cmake@1.8.0")

CPMAddPackage(
  NAME fmt
  GIT_TAG 10.1.1
  GITHUB_REPOSITORY fmtlib/fmt
  OPTIONS "FMT_INSTALL YES" # create an installable target
)

add_compile_definitions(GLM_ENABLE_EXPERIMENTAL)

CPMAddPackage(
  NAME glm
  GIT_TAG 1.0.1
  GITHUB_REPOSITORY g-truc/glm
)

CPMAddPackage(
  NAME nlohmann_json
  VERSION 3.11.2
  # the git repo is incredibly large, so we download the archived include directory
  URL https://github.com/nlohmann/json/releases/download/v3.11.2/include.zip
  URL_HASH SHA256=e5c7a9f49a16814be27e4ed0ee900ecd0092bfb7dbfca65b5a421b774dccaaed
)

if (nlohmann_json_ADDED)
  add_library(nlohmann_json INTERFACE IMPORTED)
  target_include_directories(nlohmann_json INTERFACE ${nlohmann_json_SOURCE_DIR}/include)
endif()

CPMAddPackage(
  NAME args
  GIT_TAG 6.4.7
  GITHUB_REPOSITORY Taywee/args
)

# Source file grouping of visual studio and xcode
CPMAddPackage(
  NAME GroupSourcesByFolder.cmake
  GITHUB_REPOSITORY TheLartians/GroupSourcesByFolder.cmake
  VERSION 1.0
)

# ---- Add source files ----

# Note: globbing sources is considered bad practice as CMake's generators may not detect new files
# automatically. Keep that in mind when changing files, or explicitly mention them here.
file(GLOB_RECURSE headers CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/include/*.h")
file(GLOB_RECURSE sources CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/*")

# ---- Create standalone executable ----

add_executable(${PROJECT_NAME} ${sources})
GroupSourcesByFolder(${PROJECT_NAME})

if(MSVC)
  target_compile_definitions(${PROJECT_NAME} PUBLIC -DNOMINMAX -DWIN32_LEAN_AND_MEAN)
endif()

set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 20 OUTPUT_NAME "v-img")
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(${PROJECT_NAME} fmt::fmt glm nlohmann_json args)