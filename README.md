# MeshDenoiser

A cross-platform mesh normal denoising tool based on the **Static/Dynamic Filtering** algorithm
([Zhang et al., arXiv:1712.03574](https://arxiv.org/abs/1712.03574)).

Smooths noisy 3D meshes while preserving geometric detail — useful for cleaning up
photogrammetry output, 3D scans, and other noisy mesh data.

## Features

- **Multi-format input:** OBJ, PLY, OFF, STL, glTF (.gltf/.glb), USD (.usd/.usda/.usdc/.usdz)
- **Multi-format output:** Any format supported by OpenMesh (OBJ, PLY, OFF, STL, etc.)
- **Flexible CLI:** Run with built-in defaults or a custom options file
- **Pipeline timing** printed to stdout; optional JSON/CSV metrics export
- **Deterministic mode** for reproducible results
- **Optional CHOLMOD** solver backend (auto-detected, falls back to Eigen LDLT)
- **OpenMP** parallelization (optional)
- **Input validation** — rejects meshes with NaN/Inf coordinates

## Quick Start

```bash
# Denoise with built-in defaults
MeshDenoiser input.obj output.obj

# With a custom options file
MeshDenoiser options.txt input.obj output.obj

# Denoise a glTF or USD file
MeshDenoiser scan.glb denoised.obj
MeshDenoiser scene.usdz clean.ply

# Generate a default options template
MeshDenoiser --write-default-options my_options.txt

# Show all options
MeshDenoiser --help
```

### Optional flags

| Flag | Description |
|------|-------------|
| `--export-precision N` | Vertex coordinate precision for output (default: 16) |
| `--metrics-json PATH` | Write JSON timing and solver metrics |
| `--metrics-csv PATH` | Append CSV timing and solver metrics |
| `--deterministic` | Force single-threaded execution for reproducible output |
| `--write-default-options PATH` | Write the default options template to a file and exit |

## Denoising Parameters

The defaults are tuned for detail-preserving cleanup. Generate a commented template with
`MeshDenoiser --write-default-options options.txt`, or see `MeshDenoiserDefaults.txt`.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `Lambda` | 0.15 | Regularization weight. Higher = more smoothing per iteration |
| `Eta` | 2.2 | Spatial Gaussian sigma, scaled by average face-centroid distance |
| `Mu` | 0.2 | Guidance normal difference weight |
| `Nu` | 0.25 | Signal normal difference weight |
| `MeshUpdateClosenessWeight` | 0.001 | Vertex position fidelity during mesh update |
| `MeshUpdateIterations` | 20 | Max iterations for mesh vertex update per outer iteration |
| `MeshUpdateDisplacementEps` | 0.1 | Early-stop threshold for mesh update RMS displacement (<=0 disables) |
| `OuterIterations` | 1 | Number of full filtering passes. More = more smoothing |
| `DeterministicMode` | 0 | Force single-threaded execution (0/1) |
| `LinearSolverType` | 1 | 0=CG, 1=Eigen LDLT, 2=CHOLMOD (falls back to 1 if unavailable) |

## Dependencies

| Library | Version | Type | Notes |
|---------|---------|------|-------|
| **Eigen** | 3.3+ | Header-only | Must be findable via `find_package(Eigen3)` or `-DEIGEN3_INCLUDE_DIR=...` |
| **OpenMesh** | 11.0 | Fetched automatically | Mesh data structure and traditional format I/O |
| **tinygltf** | 2.9.7 | Fetched automatically | Header-only glTF 2.0 parser |
| **tinyusdz** | 0.9.1 | Fetched automatically | USD format support |
| **OpenMP** | — | Optional | Multi-threaded performance (auto-detected) |
| **SuiteSparse CHOLMOD** | — | Optional | Faster sparse solver (auto-detected, falls back to Eigen LDLT) |

## Build

### Linux (Ubuntu/Debian)
```bash
sudo apt-get update && sudo apt-get install -y build-essential cmake libeigen3-dev
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
./build/MeshDenoiser --help
```

### macOS
```bash
brew update && brew install cmake eigen
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
./build/MeshDenoiser --help
```

For OpenMP on macOS, install `gcc` and configure with `-DCMAKE_C_COMPILER=gcc-14 -DCMAKE_CXX_COMPILER=g++-14`.

### Windows (MSVC + vcpkg)
```powershell
vcpkg install eigen3
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE="C:/path/to/vcpkg/scripts/buildsystems/vcpkg.cmake"
cmake --build build --config Release
```

## Input Format Notes

- For **glTF** and **USD** files with multiple meshes or transforms, all geometry is combined and transforms are applied automatically before filtering.
- Output format is determined by file extension (e.g., `.obj`, `.ply`, `.stl`).

## CI / Pre-built Binaries

GitHub Actions builds and uploads artifacts for Ubuntu, macOS, and Windows.
Download from the [Releases page](../../releases).

### macOS Security Note

Downloaded binaries may be quarantined. To fix:
```bash
xattr -cr meshdenoiser-macos/
```

## Licensing

- **MeshSDFilter algorithm** is BSD-3-Clause (see `LICENSES/MeshSDFilter-BSD-3-Clause.txt`).
- **OpenMesh** license: `LICENSES/OpenMesh-LICENSE.txt`.
- **tinygltf** license: `LICENSES/tinygltf-LICENSE.txt`.
- This project is licensed under the Mozilla Public License v2.0.
