cmake_minimum_required(VERSION 3.20)
project(talon_wasm LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Eigen, header-only, same version the native build fetches.
include(FetchContent)
set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_PKGCONFIG OFF CACHE BOOL "" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
    eigen
    URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
)
FetchContent_MakeAvailable(eigen)

# Single translation unit: exports.cpp includes the header-only kernel inline.
add_executable(talon.wasm src/exports.cpp)
target_link_libraries(talon.wasm PRIVATE Eigen3::Eigen)
target_include_directories(talon.wasm PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../core/include)

# Emscripten link flags for embind. -s flags go through target_link_options.
target_link_options(talon.wasm PRIVATE
    "SHELL:-s WASM=1"
    "SHELL:-s ALLOW_MEMORY_GROWTH=1"
    "SHELL:-s MODULARIZE=1"
    "SHELL:-s EXPORT_NAME=talonKernel"
    "SHELL:-s NO_EXIT_RUNTIME=1"
    "SHELL:-s ENVIRONMENT=web,node"
    "--bind"
)
