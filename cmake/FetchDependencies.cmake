# Fetch external dependencies for MeshDenoiser.
include(FetchContent)

if(POLICY CMP0169)
  cmake_policy(SET CMP0169 OLD)
endif()

if(NOT DEFINED CMAKE_POLICY_VERSION_MINIMUM OR CMAKE_POLICY_VERSION_MINIMUM VERSION_LESS 3.5)
  set(CMAKE_POLICY_VERSION_MINIMUM 3.5)
endif()

# ---------- OpenMesh ----------
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

# ---------- tinygltf (header-only) ----------
set(_TINYGLTF_URL "https://github.com/syoyo/tinygltf/archive/refs/tags/v2.9.7.zip")
FetchContent_Declare(
  tinygltf
  URL "${_TINYGLTF_URL}"
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  URL_HASH SHA256=1015f306721257fdcee602c2995542232042b4feda2ebb2e0c323c7d769ccd0e
)
FetchContent_GetProperties(tinygltf)
if(NOT tinygltf_POPULATED)
  FetchContent_Populate(tinygltf)
endif()

# ---------- tinyusdz ----------
set(_TINYUSDZ_URL "https://github.com/lighttransport/tinyusdz/archive/refs/tags/v0.9.1.zip")

set(TINYUSDZ_PRODUCTION_BUILD ON CACHE BOOL "" FORCE)
set(TINYUSDZ_WITH_OPENSUBDIV OFF CACHE BOOL "" FORCE)
set(TINYUSDZ_WITH_AUDIO OFF CACHE BOOL "" FORCE)
set(TINYUSDZ_WITH_EXR OFF CACHE BOOL "" FORCE)
set(TINYUSDZ_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(TINYUSDZ_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(TINYUSDZ_NO_WERROR ON CACHE BOOL "" FORCE)

FetchContent_Declare(
  tinyusdz
  URL "${_TINYUSDZ_URL}"
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  URL_HASH SHA256=0cdbb15147783b916fb4a487a283495d271724c101df3c98303de96a3ef1af66
)
FetchContent_MakeAvailable(tinyusdz)

# ---------- Optional: SuiteSparse CHOLMOD ----------
option(ENABLE_CHOLMOD "Enable SuiteSparse CHOLMOD backend when available" ON)
set(SDFILTER_ENABLE_CHOLMOD OFF)
set(SDFILTER_CHOLMOD_LINK_LIBS "")
set(SDFILTER_CHOLMOD_INCLUDE_DIRS "")

if(ENABLE_CHOLMOD)
  find_package(CHOLMOD CONFIG QUIET)
  if(CHOLMOD_FOUND OR TARGET CHOLMOD::CHOLMOD)
    set(SDFILTER_ENABLE_CHOLMOD ON)
    message(STATUS "CHOLMOD found. Enabling CHOLMOD linear-solver backend.")
    if(TARGET CHOLMOD::CHOLMOD)
      set(SDFILTER_CHOLMOD_LINK_LIBS CHOLMOD::CHOLMOD)
    elseif(DEFINED CHOLMOD_LIBRARIES)
      set(SDFILTER_CHOLMOD_LINK_LIBS ${CHOLMOD_LIBRARIES})
    endif()
    if(DEFINED CHOLMOD_INCLUDE_DIRS)
      set(SDFILTER_CHOLMOD_INCLUDE_DIRS ${CHOLMOD_INCLUDE_DIRS})
    elseif(DEFINED CHOLMOD_INCLUDE_DIR)
      set(SDFILTER_CHOLMOD_INCLUDE_DIRS ${CHOLMOD_INCLUDE_DIR})
    endif()
  else()
    find_package(SuiteSparse QUIET)
    if(SuiteSparse_FOUND)
      set(SDFILTER_ENABLE_CHOLMOD ON)
      message(STATUS "SuiteSparse found. Enabling CHOLMOD linear-solver backend.")
      if(TARGET SuiteSparse::CHOLMOD)
        set(SDFILTER_CHOLMOD_LINK_LIBS SuiteSparse::CHOLMOD)
      elseif(DEFINED CHOLMOD_LIBRARIES)
        set(SDFILTER_CHOLMOD_LINK_LIBS ${CHOLMOD_LIBRARIES})
      endif()
      if(DEFINED SuiteSparse_INCLUDE_DIRS)
        set(SDFILTER_CHOLMOD_INCLUDE_DIRS ${SuiteSparse_INCLUDE_DIRS})
      elseif(DEFINED CHOLMOD_INCLUDE_DIRS)
        set(SDFILTER_CHOLMOD_INCLUDE_DIRS ${CHOLMOD_INCLUDE_DIRS})
      endif()
    else()
      message(STATUS "SuiteSparse/CHOLMOD not found. CHOLMOD backend disabled; using Eigen LDLT.")
    endif()
  endif()
endif()
