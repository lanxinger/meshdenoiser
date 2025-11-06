cmake_minimum_required(VERSION 3.18)
project(meshsdfilter-builder LANGUAGES CXX)

# Set C++17 standard for filesystem support
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED OFF)
set(CMAKE_CXX_EXTENSIONS OFF)

include(FetchContent)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Option: enable/disable OpenMP (MeshSDFilter uses it if available)
option(ENABLE_OPENMP "Enable OpenMP if available" ON)

# --- Fetch the AliceVision source at the requested commit ---
set(ALICEVISION_COMMIT "14b0b8f8b3026765d165dfc3f219ed8c53635f52")
include(FetchMeshSDFilter)

# On MSVC, use static runtime if desired (optional)
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

if(ENABLE_OPENMP)
  find_package(OpenMP)
  if(OpenMP_CXX_FOUND)
    message(STATUS "OpenMP found: enabling for targets built in MeshSDFilter")
  endif()
endif()
