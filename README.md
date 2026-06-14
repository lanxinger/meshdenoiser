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

## Using from Swift (iOS / macOS)

This repo is also a Swift Package (`MeshDenoiserKit`) targeting iOS 17+ / macOS 14+.
It exposes the denoiser as a buffers-in/buffers-out API — file I/O stays in your app
(ModelIO/SceneKit read USDZ natively). Vertex count and order are preserved, so UVs
and materials on the original asset remain valid; recompute normals after denoising.

```swift
import MeshDenoiserKit

var params = MeshDenoiseParameters()        // tuned defaults
params.outerIterations = 2                  // more smoothing
params.backend = .reference                 // default while native backend matures

let denoised = try await MeshDenoiser.denoise(
    positions: positions,                   // [SIMD3<Float>]
    indices: indices,                       // [UInt32], 3 per triangle
    parameters: params
) { progress in
    print("Progress: \(progress)")
}
```

Cancellation: cancel the surrounding `Task`; the call throws `CancellationError`
(checked after each outer iteration). For very large meshes (>~500k vertices) set
`params.linearSolver = .conjugateGradient` to reduce memory use.

On macOS, app code that works with USD/USDZ assets can use the ModelIO/SceneKit
adapter directly:

```swift
import MeshDenoiserKit

var params = MeshDenoiseParameters()
params.backend = .automatic

let options = MeshAssetDenoiseOptions(
    parameters: params,
    preprocessing: .none
)

let preflight = try MeshAssetDenoiser.preflight(
    inputURL: inputUSDZ,
    options: options
)

let preview = try await MeshAssetDenoiser.preview(
    inputURL: inputUSDZ,
    options: options
) { progress in
    print("Preview progress: \(progress)")
}

let denoisedPositions = preview.meshes[0].denoisedPositions

let summary = try await MeshAssetDenoiser.process(
    inputURL: inputUSDZ,
    outputURL: outputUSDZ,
    options: options
) { progress in
    print("Progress: \(progress)")
}

// If the app already has an MDLAsset loaded, avoid a second read:
let inMemorySummary = try await MeshAssetDenoiser.process(
    asset: modelIOAsset,
    outputURL: outputUSDZ,
    options: options
)
```

`MeshAssetDenoiser.preview(...)` denoises each `MDLMesh` and returns original
positions, denoised positions, and shared triangle indices without writing or
mutating the source asset. Use it for interactive app previews where exporting
a full USDZ would make the UI wait on asset packaging.

`MeshAssetDenoiser.process(...)` writes a new denoised asset, preserving vertex
count/order, submeshes, materials, UVs, and transforms. USDZ output patches the
USD geometry and repacks the asset because ModelIO can read USDZ but does not
export it directly. The URL-based `process(inputURL:outputURL:...)` API requires
distinct input and output URLs so a failed export cannot overwrite the source
asset. Asset export writes to a temporary sibling file first and replaces the
final output only after export succeeds. Progress callbacks are monotonic and
cover `0...1`.
`preflight(...)` returns the same mesh counts and preprocessing diagnostics
without denoising or exporting, which is useful for showing validation failures
before starting the processing step.
Topology-changing repair is not applied inside this asset adapter; use
`.conservativeValidation(...)` only when you want to reject assets that would
need face/topology changes before denoising. Validation allows unused-vertex
compaction diagnostics because the adapter keeps the original vertex buffer
shape and denoises referenced and unreferenced vertices in place.

Swift backend selection:

| Backend | Status |
|---------|--------|
| `.reference` | Default C++/OpenMesh/Eigen backend; exact golden parity target |
| `.nativeCPU` | Native Swift pipeline with CPU fixed-point filter and CPU vertex update |
| `.nativeGPU` | Native Swift pipeline with Metal fixed-point filter; preprocessing and vertex update stay on CPU |
| `.automatic` | Uses native CPU by default; explicit `.nativeGPU` is available for benchmarked GPU runs |

The native backends are under active parity/performance validation. Keep `.reference`
as the default for production until benchmark gates and larger fixture coverage pass.

Benchmark Swift backends with:

```bash
swift run -c release MeshDenoiserBench --faces 81920 --backend all
```

Use a local USD/USDZ or OBJ file instead of a generated sphere with:

```bash
swift run -c release MeshDenoiserBench --input path/to/model.usdz --backend all
```

Exercise the same URL-based asset processing API used by macOS app code with:

```bash
swift run -c release MeshDenoiserProcess --input path/to/model.usdz --output path/to/denoised.usdz --backend automatic
swift run -c release MeshDenoiserProcess --input path/to/model.usdz --output path/to/denoised.usdz --backend nativeCPU --repair conservative
```

`MeshDenoiserProcess` prints `progress=...` lines to stderr and a compact
`meshes,vertices,faces` CSV summary to stdout after export succeeds.

Run the same benchmark across a folder or manifest of app-exported assets with:

```bash
swift run -c release MeshDenoiserBench --input-dir path/to/usdz-fixtures --backend all
swift run -c release MeshDenoiserBench --input-list path/to/fixtures.txt --backend all
swift run -c release MeshDenoiserBench --input-dir path/to/usdz-fixtures --backend all --keep-going
swift run -c release MeshDenoiserBench --input-dir path/to/usdz-fixtures --backend all --keep-going --fail-on-error
swift run -c release MeshDenoiserBench --input-dir path/to/usdz-fixtures --backend all --repair conservative --keep-going
swift run -c release MeshDenoiserBench --input-dir path/to/usdz-fixtures --backend all --keep-going --fail-on-error --max-error-threshold 0.002 --mean-error-threshold 0.0002 --max-total-seconds-threshold 10 --max-memory-mb-threshold 8192
swift run -c release MeshDenoiserBench --input-dir path/to/usdz-fixtures --backend nativeGPU --repair conservative --no-reference-check --keep-going --fail-on-error --max-total-seconds-threshold 10 --max-memory-mb-threshold 8192
```

The benchmark importer uses ModelIO on macOS for `.usd`, `.usda`, `.usdc`, and
`.usdz`, flattens mesh transforms, and triangulates supported submeshes before
calling the buffers-in API. Directory and manifest runs add an `input` column to
the CSV output so production USDZ fixture results can be compared asset by asset.
Use `--keep-going` during broad fixture sweeps to keep benchmarking after an
asset fails validation; those rows add `status,error` columns and include simple
mesh diagnostics such as boundary, non-manifold, and degenerate-face counts.
Add `--fail-on-error` in CI to complete the sweep and then exit nonzero if any
asset/backend row failed. Add `--max-error-threshold` and
`--mean-error-threshold` to make successful native rows fail the sweep when
reference parity exceeds the release gate. Add `--max-total-seconds-threshold`
and `--max-memory-mb-threshold` to enforce runtime and peak-RSS budgets on the
same batch pass. Use `--no-reference-check` for production runtime and memory
gates on large USDZ assets when you do not want the benchmark-only C++ reference
comparison to affect peak RSS; error thresholds are only valid when reference
checking is enabled.

Use `--repair conservative` to opt into an explicit mesh preflight before
denoising. The conservative pass welds duplicate or near-duplicate vertices,
removes degenerate and duplicate triangles, drops extra faces around
non-manifold edges, orients shared edges consistently, and compacts unused
vertices. It does not fill holes or silently run by default. Repair runs add
per-row CSV diagnostics describing before/after vertex, face, boundary-edge,
non-manifold-edge, and degenerate-face counts, plus removed face and vertex
counts.

CSV output columns are:

```text
backend,vertices,faces,total_secs,max_error_vs_reference,mean_error_vs_reference,peak_memory_mb
```

With `--no-reference-check`, native denoise rows omit the reference-error
columns:

```text
backend,vertices,faces,total_secs,peak_memory_mb
```

Benchmark the native vertex-update stage directly with:

```bash
swift run -c release MeshDenoiserBench --mode filter --backend nativeGPU --faces 81920
swift run -c release MeshDenoiserBench --mode vertex-update --faces 81920
```

The filter and vertex-update modes use deterministic generated meshes by
default, accept `--input`, and print:

```text
mode,backend,vertices,faces,total_secs,peak_memory_mb,checksum
```

Vendored dependencies (Eigen, OpenMesh Core) are pinned by `scripts/vendor_dependencies.sh`.

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
