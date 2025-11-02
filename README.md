# meshsdfilter-builder

A tiny, cross‑platform CMake repo that **uses the MeshSDFilter code from AliceVision** at commit
`14b0b8f8b3026765d165dfc3f219ed8c53635f52` and builds the two binaries:

- `MeshSDFilter`
- `MeshDenoiser`

Dependencies are handled per‑platform (OpenMesh sources are bundled by the upstream code; Eigen is installed by the workflow/package manager).

## Dependencies

MeshSDFilter requires:
- **Eigen 3.3+** (header-only)
- **OpenMP** (optional, if compiler supports it)
- **OpenMesh** sources – the upstream MeshSDFilter CMake builds a static OpenMesh from its bundled `external/OpenMesh` folder.
- **tinygltf** (header-only, fetched automatically) to allow loading `.gltf` and `.glb` meshes when running `MeshDenoiser`.

OpenMP is an open standard for shared-memory parallelism; compilers that support it (e.g. GCC, Clang with `libomp`, MSVC) let MeshSDFilter run heavy loops across multiple CPU cores.

> Note: MeshSDFilter expects Eigen to be discoverable via `find_package(Eigen3)`. Our CI installs Eigen per-platform so you don’t have to.
> You can also point CMake at a local Eigen install with `-DEIGEN3_INCLUDE_DIR=/path/to/eigen` if needed.

## Build (local)

### Linux (Ubuntu/Debian)
```bash
sudo apt-get update && sudo apt-get install -y build-essential cmake libeigen3-dev
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
./build/MeshSDFilter --help
./build/MeshDenoiser --help
```

### macOS
Using Homebrew:
```bash
brew update && brew install cmake eigen
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
./build/MeshSDFilter --help
./build/MeshDenoiser --help
```
If you want OpenMP on macOS, install `gcc` and configure CMake with `-DCMAKE_C_COMPILER=gcc-14 -DCMAKE_CXX_COMPILER=g++-14` (or the version you installed).

### Windows (MSVC + vcpkg)
```powershell
# One-time: install vcpkg and integrate
# https://github.com/microsoft/vcpkg#quick-start-windows
vcpkg install eigen3

# Configure with vcpkg toolchain so find_package(Eigen3) works
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE="C:/path/to/vcpkg/scripts/buildsystems/vcpkg.cmake"
cmake --build build --config Release
# Binaries will be under build/Release/
```

## CI
- GitHub Actions build and upload artifacts for Ubuntu, macOS, Windows.
- Artifacts include the two binaries per platform (`MeshSDFilter`, `MeshDenoiser`).

## Licensing
- **MeshSDFilter** code is BSD-3-Clause (see `LICENSES/MeshSDFilter-BSD-3-Clause.txt`).
- **OpenMesh** license is included as `LICENSES/OpenMesh-LICENSE.txt`.
- This wrapper repo is MIT by default (you can change it), and preserves upstream notices.

## Using the tools
```bash
# Filter
MeshSDFilter FilteringOptions.txt input_mesh.ply output_mesh.ply
# Denoise
MeshDenoiser DenoisingOptions.txt input_mesh.ply output_mesh.ply
```
- A detail-preserving MeshDenoiser preset is in `MeshDenoiserDefaults.txt` (outer iterations 1, lambda 0.15, eta 2.2, mu 0.2, nu 0.25). Copy it to your working folder or pass it directly; raise lambda/eta or the iteration count only if you want stronger smoothing.
- `MeshDenoiser` accepts OBJ, PLY, OFF, STL, and now `.gltf/.glb` inputs (thanks to tinygltf). If the glTF mesh contains multiple nodes, transforms are applied automatically before filtering.
