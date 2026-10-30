# Fetch external dependencies for MeshDenoiser.
include(FetchContent)

if(NOT DEFINED CMAKE_POLICY_VERSION_MINIMUM OR CMAKE_POLICY_VERSION_MINIMUM VERSION_LESS 3.5)
  set(CMAKE_POLICY_VERSION_MINIMUM 3.5)
endif()

# ---------- OpenMesh ----------
set(_OPENMESH_URL "https://www.graphics.rwth-aachen.de/media/openmesh_static/Releases/11.0/OpenMesh-11.0.0.tar.gz")
set(BUILD_APPS OFF CACHE BOOL "" FORCE)
set(OPENMESH_DOCS OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
  openmesh
  URL "${_OPENMESH_URL}"
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  URL_HASH SHA256=c7f35d29673e6dbb6d65b214c10c4c6249521a8f1e8f8db6e8bdc2eed798aedc
)
FetchContent_MakeAvailable(openmesh)

# ---------- tinygltf (C11 runtime) ----------
# Only the single-file C runtime is compiled (see CMakeLists.txt); SOURCE_SUBDIR points at a
# directory without a CMakeLists.txt so the project's own tests and examples are not added.
set(_TINYGLTF_URL "https://github.com/syoyo/tinygltf/archive/refs/tags/v3.0.1.zip")
FetchContent_Declare(
  tinygltf
  URL "${_TINYGLTF_URL}"
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  URL_HASH SHA256=0814d73c8857b99af1597342752af6c47f70c7a686fe832b5465e27029ce8857
  SOURCE_SUBDIR no-cmake-project
)
FetchContent_MakeAvailable(tinygltf)

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
  URL_HASH SHA256=d31346550239c862cace6728214b60ec3842702f91ea02e537852d9388ec9bac
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
