cmake_minimum_required(VERSION 3.14...3.22)

# ---- Project ----

# Note: update this to your new project's name and version
project(Greeter LANGUAGES CXX CUDA)

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
  GIT_TAG 9.1.0
  GITHUB_REPOSITORY fmtlib/fmt
  OPTIONS "FMT_INSTALL YES" # create an installable target
)

# ---- Add source files ----

# Note: globbing sources is considered bad practice as CMake's generators may not detect new files
# automatically. Keep that in mind when changing files, or explicitly mention them here.
file(GLOB_RECURSE headers CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/include/*.h")
file(GLOB_RECURSE sourcescu CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cu")
file(GLOB_RECURSE sourcescpp CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")

# ---- Create standalone executable ----

add_executable(${PROJECT_NAME} ${sourcescu} ${sourcescpp})
set_target_properties(${PROJECT_NAME} PROPERTIES CUDA_ARCHITECTURES native)
set_target_properties(${PROJECT_NAME} PROPERTIES CUDA_SEPARABLE_COMPILATION ON OUTPUT_NAME "Greeter")
target_link_libraries(${PROJECT_NAME} fmt::fmt)