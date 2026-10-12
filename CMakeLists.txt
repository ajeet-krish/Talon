cmake_minimum_required(VERSION 3.20)
project(talon LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(BUILD_TESTS "Build C++ unit tests" ON)
option(BUILD_BINDINGS "Build the pybind11 Python module (_core)" ON)

include(FetchContent)
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.14.0
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_Declare(
    eigen
    URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
)
FetchContent_Declare(
    pybind11
    GIT_REPOSITORY https://github.com/pybind/pybind11.git
    GIT_TAG v3.0.1
)

# Header-only interface library: the entire C++ kernel.
add_library(talon_headers INTERFACE)
target_include_directories(talon_headers
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/cpp/include
)
target_compile_features(talon_headers INTERFACE cxx_std_17)

if(BUILD_TESTS)
    set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)
endif()

set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_PKGCONFIG OFF CACHE BOOL "" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(eigen)
target_link_libraries(talon_headers INTERFACE Eigen3::Eigen)

if(MSVC)
    target_compile_options(talon_headers INTERFACE /O2)
else()
    target_compile_options(talon_headers INTERFACE -O2)
endif()

if(BUILD_TESTS)
    enable_testing()
    add_executable(talon_tests
        cpp/tests/test_controller.cpp
        cpp/tests/test_dynamics.cpp
        cpp/tests/test_wind.cpp
    )
    target_link_libraries(talon_tests PRIVATE talon_headers GTest::gtest_main)
    include(GoogleTest)
    gtest_discover_tests(talon_tests)
endif()

if(BUILD_BINDINGS)
    # Prefer the active virtualenv interpreter so the built extension matches
    # the one `uv run` uses (otherwise FindPython may pick the system python).
    if(DEFINED ENV{VIRTUAL_ENV} AND NOT DEFINED Python_EXECUTABLE)
        set(Python_EXECUTABLE "$ENV{VIRTUAL_ENV}/bin/python")
    endif()

    FetchContent_MakeAvailable(pybind11)
    pybind11_add_module(_core bindings/pytalon.cpp)
    target_link_libraries(_core PRIVATE talon_headers)

    # Land the module inside the Python package so `import talon._core` works.
    set(_talon_pkg_dir ${CMAKE_CURRENT_SOURCE_DIR}/src/talon)
    set_target_properties(_core PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY ${_talon_pkg_dir}
        RUNTIME_OUTPUT_DIRECTORY ${_talon_pkg_dir}
    )
    foreach(cfg IN ITEMS Debug Release RelWithDebInfo MinSizeRel)
        string(TOUPPER ${cfg} cfg_upper)
        set_target_properties(_core PROPERTIES
            LIBRARY_OUTPUT_DIRECTORY_${cfg_upper} ${_talon_pkg_dir}
            RUNTIME_OUTPUT_DIRECTORY_${cfg_upper} ${_talon_pkg_dir}
        )
    endforeach()
endif()
