# Fetch the AliceVision tarball at a specific commit and add only MeshSDFilter.
include(FetchContent)

if(POLICY CMP0169)
  cmake_policy(SET CMP0169 OLD)
endif()

if(NOT DEFINED CMAKE_POLICY_VERSION_MINIMUM OR CMAKE_POLICY_VERSION_MINIMUM VERSION_LESS 3.5)
  set(CMAKE_POLICY_VERSION_MINIMUM 3.5)
endif()

if(NOT ALICEVISION_COMMIT)
  message(FATAL_ERROR "ALICEVISION_COMMIT must be set before including FetchMeshSDFilter.cmake")
endif()

set(_AV_URL "https://github.com/alicevision/AliceVision/archive/${ALICEVISION_COMMIT}.zip")
FetchContent_Declare(
  alicevision_src
  URL "${_AV_URL}"
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  URL_HASH SHA256=9f3d1718e5cff1fc81a21d2a498557c342913208d8479cf2565075cc0be71d9b
)
FetchContent_GetProperties(alicevision_src)
if(NOT alicevision_src_POPULATED)
  FetchContent_Populate(alicevision_src)
endif()

set(_OPENMESH_URL "https://www.graphics.rwth-aachen.de/media/openmesh_static/Releases/11.0/OpenMesh-11.0.0.tar.gz")
FetchContent_Declare(
  openmesh
  URL "${_OPENMESH_URL}"
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_GetProperties(openmesh)
if(NOT openmesh_POPULATED)
  FetchContent_Populate(openmesh)
  set(BUILD_APPS OFF CACHE BOOL "" FORCE)
  set(OPENMESH_DOCS OFF CACHE BOOL "" FORCE)
  add_subdirectory("${openmesh_SOURCE_DIR}" "${openmesh_BINARY_DIR}")
endif()

set(OpenMesh_DIR "${openmesh_BINARY_DIR}/src/cmake" CACHE PATH "" FORCE)
list(PREPEND CMAKE_PREFIX_PATH "${openmesh_BINARY_DIR}")
list(PREPEND CMAKE_MODULE_PATH "${openmesh_SOURCE_DIR}/cmake")

set(_TINYGLTF_URL "https://github.com/syoyo/tinygltf/archive/refs/tags/v2.9.6.zip")
FetchContent_Declare(
  tinygltf
  URL "${_TINYGLTF_URL}"
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  URL_HASH SHA256=a3eade1a5f00a756f81d43e1ef6b23c3bde9cda0c52a7a69cca5dd6ad8f1dd95
)
FetchContent_GetProperties(tinygltf)
if(NOT tinygltf_POPULATED)
  FetchContent_Populate(tinygltf)
endif()

set(_TINYUSDZ_URL "https://github.com/lighttransport/tinyusdz/archive/refs/heads/release.zip")
FetchContent_Declare(
  tinyusdz
  URL "${_TINYUSDZ_URL}"
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_GetProperties(tinyusdz)
if(NOT tinyusdz_POPULATED)
  FetchContent_Populate(tinyusdz)
  # TinyUSDZ needs to be built as a library
  set(TINYUSDZ_PRODUCTION_BUILD ON CACHE BOOL "" FORCE)
  set(TINYUSDZ_WITH_OPENSUBDIV OFF CACHE BOOL "" FORCE)
  set(TINYUSDZ_WITH_AUDIO OFF CACHE BOOL "" FORCE)
  set(TINYUSDZ_WITH_EXR OFF CACHE BOOL "" FORCE)
  set(TINYUSDZ_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(TINYUSDZ_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  add_subdirectory("${tinyusdz_SOURCE_DIR}" "${tinyusdz_BINARY_DIR}")
endif()

# Export tinyusdz include directory for use by MeshSDFilter
set(TINYUSDZ_INCLUDE_DIR "${tinyusdz_SOURCE_DIR}/src" CACHE INTERNAL "")

set(MESHSD_DIR "${alicevision_src_SOURCE_DIR}/src/dependencies/MeshSDFilter")

if(NOT EXISTS "${MESHSD_DIR}/CMakeLists.txt")
  message(FATAL_ERROR "MeshSDFilter CMakeLists.txt not found at ${MESHSD_DIR}. Commit may have changed structure.")
endif()

set(_override_dir "${PROJECT_SOURCE_DIR}/overrides/MeshSDFilter")
message(STATUS "Checking override dir: ${_override_dir}")
if(EXISTS "${_override_dir}")
  file(GLOB _override_sources "${_override_dir}/*")
  foreach(_src IN LISTS _override_sources)
    get_filename_component(_fname "${_src}" NAME)
    message(STATUS "Overriding MeshSDFilter source: ${_fname}")
    configure_file("${_src}" "${MESHSD_DIR}/${_fname}" COPYONLY)
  endforeach()
endif()

add_subdirectory("${MESHSD_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/MeshSDFilter-build")

foreach(_mesh_target MeshSDFilter MeshDenoiser)
  if(TARGET ${_mesh_target})
    target_include_directories(${_mesh_target} PRIVATE "${tinygltf_SOURCE_DIR}")
    target_include_directories(${_mesh_target} PRIVATE "${TINYUSDZ_INCLUDE_DIR}")
    target_link_libraries(${_mesh_target} tinyusdz_static)
  endif()
endforeach()
