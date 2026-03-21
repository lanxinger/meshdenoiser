cmake_minimum_required(VERSION 3.18)
project(MeshDenoiser LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(FetchContent)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Compiler flags (applied per-target below, not globally, to avoid breaking fetched deps)

# OpenMP (optional)
option(ENABLE_OPENMP "Enable OpenMP if available" ON)
if(ENABLE_OPENMP)
  find_package(OpenMP QUIET)
endif()

# Eigen (required, header-only)
find_package(Eigen3 3.3 QUIET)
if(NOT TARGET Eigen3::Eigen)
  # Retry without version constraint
  find_package(Eigen3 QUIET)
endif()
if(NOT TARGET Eigen3::Eigen)
  # Try user-provided include dir
  if(DEFINED EIGEN3_INCLUDE_DIR AND EXISTS "${EIGEN3_INCLUDE_DIR}")
    set(_eigen_dir "${EIGEN3_INCLUDE_DIR}")
  else()
    # Probe common system locations
    find_path(_eigen_dir NAMES Eigen/Dense
      PATHS /usr/include/eigen3 /usr/local/include/eigen3 /opt/homebrew/include/eigen3
      NO_DEFAULT_PATH)
    if(NOT _eigen_dir)
      find_path(_eigen_dir NAMES Eigen/Dense)
    endif()
  endif()
  if(_eigen_dir)
    message(STATUS "Using Eigen from ${_eigen_dir}")
    add_library(Eigen3::Eigen INTERFACE IMPORTED)
    set_target_properties(Eigen3::Eigen PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${_eigen_dir}")
  else()
    message(FATAL_ERROR "Eigen3 not found. Install via system package manager or set -DEIGEN3_INCLUDE_DIR=...")
  endif()
endif()

# Fetch OpenMesh, tinygltf, tinyusdz, and optionally detect CHOLMOD
include(FetchDependencies)

# On MSVC, optionally link static runtime
option(USE_STATIC_MSVC_RUNTIME "Link static MSVC runtime (/MT)" OFF)
if(MSVC AND USE_STATIC_MSVC_RUNTIME)
  foreach(flag_var CMAKE_C_FLAGS CMAKE_C_FLAGS_DEBUG CMAKE_C_FLAGS_RELEASE
                   CMAKE_C_FLAGS_RELWITHDEBINFO CMAKE_C_FLAGS_MINSIZEREL
                   CMAKE_CXX_FLAGS CMAKE_CXX_FLAGS_DEBUG CMAKE_CXX_FLAGS_RELEASE
                   CMAKE_CXX_FLAGS_RELWITHDEBINFO CMAKE_CXX_FLAGS_MINSIZEREL)
    if(${flag_var} MATCHES "/MD")
      string(REGEX REPLACE "/MD" "/MT" ${flag_var} "${${flag_var}}")
    endif()
  endforeach()
endif()

# ---------- MeshDenoiser executable ----------
add_executable(MeshDenoiser
  src/EigenTypes.h
  src/MeshTypes.h
  src/SDFilter.h
  src/MeshNormalFilter.h
  src/MeshNormalDenoising.h
  src/AppMetrics.h
  src/MeshDenoiser.cpp
)

# Per-target compiler flags
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU|AppleClang")
  target_compile_options(MeshDenoiser PRIVATE -Wall -Wextra)
  target_compile_options(MeshDenoiser PRIVATE "$<$<CONFIG:Release>:-march=native;-mtune=native>")
elseif(MSVC)
  target_compile_definitions(MeshDenoiser PRIVATE USE_MSVC _USE_MATH_DEFINES)
endif()

target_include_directories(MeshDenoiser PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/src"
  "${tinygltf_SOURCE_DIR}"
  "${tinyusdz_SOURCE_DIR}/src"
)

target_link_libraries(MeshDenoiser PRIVATE
  Eigen3::Eigen
  OpenMeshCore
  tinyusdz_static
)

# OpenMP
if(OpenMP_CXX_FOUND)
  target_compile_definitions(MeshDenoiser PRIVATE USE_OPENMP)
  target_link_libraries(MeshDenoiser PRIVATE OpenMP::OpenMP_CXX)
endif()

# CHOLMOD
if(SDFILTER_ENABLE_CHOLMOD)
  target_compile_definitions(MeshDenoiser PRIVATE USE_CHOLMOD)
  if(SDFILTER_CHOLMOD_INCLUDE_DIRS)
    target_include_directories(MeshDenoiser PRIVATE ${SDFILTER_CHOLMOD_INCLUDE_DIRS})
  endif()
  if(SDFILTER_CHOLMOD_LINK_LIBS)
    target_link_libraries(MeshDenoiser PRIVATE ${SDFILTER_CHOLMOD_LINK_LIBS})
  endif()
endif()
