# Swift Metal Port Analysis Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Port the current SD-filter mesh denoiser from the C++/Eigen/OpenMesh implementation to a native Swift + Metal backend, while keeping `MeshDenoiserKit`'s public buffers-in/buffers-out API stable and using the existing C++ backend as the reference oracle until native parity is proven.

**Architecture:** Keep the current SwiftPM wrapper as the integration surface, add a new `MeshDenoiserNative` Swift target, and route calls through a backend enum. The native backend owns validation, compact mesh topology, patch-guidance normals, radius neighbor search, static filter precompute, a CPU fixed-point filter, a Metal fixed-point filter, and a CPU vertex update solve through an isolated matrix-free SPD conjugate-gradient solver.

**Tech Stack:** SwiftPM, Swift 5.9 initially with Swift 6 strict concurrency settings enabled where the package allows it, Metal compute kernels, XCTest, existing C++ reference backend. Accelerate Sparse remains a future direct-solver spike if the measured CG path becomes a bottleneck.

---

## Implementation Status

Completed:

- Tasks 1-6: native Swift target, backend routing, validation/normalization, connectivity, guidance normals, neighbor precompute, and CPU fixed-point filtering.
- Task 7: vertex update behavior and tests are implemented. The current solver is an isolated matrix-free SPD conjugate-gradient solve rather than Accelerate direct sparse factorization.
- Task 8: native CPU end-to-end backend is wired through `.nativeCPU` and covered by public API, cancellation, progress, and C++ reference tolerance tests.
- Task 9 first slice: Metal dynamic-weight and gather-normalize kernels are wired into `.nativeGPU`/`.automatic` fixed-point filtering, with CPU/GPU filter parity and public native GPU routing tests.
- Task 9 full filter slice: GPU displacement reduction is implemented, fixture-level CPU/GPU normal parity passes, and fixture-level `.nativeGPU`/`.nativeCPU` output parity passes.
- Task 10 first slice: `MeshDenoiserBench` executable target and README backend/benchmark documentation are implemented.
- Task 10 benchmark gate: release benchmarks were run at approximately 20k, 82k, and 328k generated faces, local OBJ file-backed meshes, and USD/USDZ smoke fixtures through ModelIO. Native GPU passes speed, parity, and memory gates on the latest matrix. Peak memory was reduced by consuming precompute storage during Metal upload and fusing dynamic-weight computation into the gather kernel, removing the pair and dynamic-weight Metal buffers.
- Task 10 USDZ validation workflow: `MeshDenoiserBench` can now run over `--input-dir` or `--input-list` so production USD/USDZ fixture sets can be validated in one CSV pass with an `input` column. Broad sweeps can use `--keep-going` to report invalid assets as CSV failure rows instead of aborting the batch; failed rows include simple mesh diagnostics for boundary, non-manifold, and degenerate topology. CI sweeps can add `--fail-on-error` to collect all rows and then exit nonzero if any asset/backend row failed. Parity gates can add `--max-error-threshold` and `--mean-error-threshold` to fail native rows that drift past release tolerances. Resource gates can add `--max-total-seconds-threshold` and `--max-memory-mb-threshold` to fail rows that exceed runtime or peak-RSS budgets.
- Mesh repair first slice: `MeshPreprocessor.repairForDenoising` adds an explicit conservative buffer repair pass. `MeshDenoiserBench --repair conservative` now runs that pass before benchmarking, but the public `denoise()` call remains strict by default. Repair benchmark rows now include before/after topology diagnostics and removal counts.
- Task 7 benchmark gate: current CG vertex-update solve was measured directly at approximately 20k, 82k, and 328k faces. It is not the dominant native runtime, so Accelerate sparse direct factorization is deferred unless larger real-world fixtures show the solver dominates.

Remaining:

- Decide whether to change the public default to `.automatic` after larger app-integration fixtures pass parity and memory gates.

## Repository Analysis

The repo already has the bridge needed to make this port incremental:

- `Package.swift` defines `OpenMeshCore`, `CMeshDenoiserCore`, `MeshDenoiserKit`, and `MeshDenoiserKitTests`.
- `Sources/MeshDenoiserKit/MeshDenoiser.swift` exposes one async public API and currently routes all work to the synchronous C API on a background queue.
- `Sources/CMeshDenoiserCore/CMeshDenoiserCore.cpp` validates raw buffers, builds an OpenMesh `TriMesh`, normalizes the mesh, runs `SDFilter::MeshNormalDenoising`, restores scale, and copies vertices back in original order.
- `Tests/MeshDenoiserKitTests/GoldenParityTests.swift` already gives a deterministic fixture and C++/CLI parity guard.
- `docs/superpowers/specs/2026-06-12-native-swift-metal-port-design.md` records the approved direction: native Swift + Metal is the target, C++ stays as `.reference` during validation.

The algorithm breaks into six stages:

1. Mesh normalization: center by vertex mean, scale by bounding-box diagonal, then restore by recentering the final normalized mesh before applying the original scale.
2. Connectivity and geometry: triangle normals, areas normalized to mean 1, centroids, edges, vertex-to-face adjacency, and non-boundary edge metadata.
3. Guidance normals: edge saliency, per-face patch candidates, patch normal consistency, patch average normals, and best-patch selection.
4. Neighbor precompute: scaled eta, uniform-grid radius search, static area/spatial/guidance pair weights, lambda rescale, and CSR-style face neighbor rows.
5. Fixed-point normal filter: dynamic pair weights, per-face gather, normalization, and convergence reduction.
6. Vertex update: iterative projection to target planes, sparse normal-equation solve, and RMS-displacement early stop.

The natural split is:

- Swift CPU: stages 1-4, CPU fallback for stage 5, orchestration, validation, tests, and benchmarks.
- Metal: stage 5 once CPU parity exists.
- Swift CPU: stage 6 sparse matrix-equivalent CG solve, with Accelerate direct sparse solve deferred as a measured optimization candidate.
- C++ reference: public `.reference` backend and golden-oracle tests until native output and performance are accepted.

## Source Of Truth Formulas

Use these current source locations while porting:

| Behavior | Formula / Contract | Current source |
|---|---|---|
| Public API | `denoise(positions:indices:parameters:progress:) async throws -> [SIMD3<Float>]` | `Sources/MeshDenoiserKit/MeshDenoiser.swift:26` |
| Neighbor radius | `radius = 3 * etaPrime`, accept strict `distanceSquared < radiusSquared` | `src/MeshNormalFilter.h:199` |
| Initial normals | normalized face normal per triangle | `src/MeshNormalFilter.h:304` |
| Area weights | face sector area divided by mean face area | `src/MeshNormalFilter.h:450` |
| Patch guidance | edge saliency, patch consistency, weighted patch average normal, min-score patch | `src/MeshNormalDenoising.h:175` |
| Static pair weight | `(area_i + area_j) * exp(h_guidance * |g_i-g_j|^2 + h_spatial * d^2)` | `src/SDFilter.h:753` |
| Lambda rescale | `lambda *= sum(areaWeights) / sum(areaSpatialWeights)` | `src/SDFilter.h:777` |
| Weighted init | `initSignal_i * area_i * 2 * nu^2 / lambdaPrime` | `src/SDFilter.h:449` |
| Dynamic pair weight | `staticWeight_p * exp(-0.5 / nu^2 * |signal_i-signal_j|^2)` | `src/SDFilter.h:473` |
| Filter gather | `filtered_i = normalize(weightedInit_i + sum(signal_neighbor * dynamicWeight))` | `src/SDFilter.h:482` |
| Filter convergence | `sum(area_i * |signal_i-prev_i|^2) <= sum(area) * avgDispEps^2`, max 100 iterations | `src/SDFilter.h:460` |
| Solver path | Eigen LDLT/CG wrapper; native replacement uses isolated matrix-free SPD CG | `src/SDFilter.h:533` |
| Mesh update system | `min ||AX-B||^2 + w||X-X0||^2`, `w = closeness * faceCount / vertexCount` | `src/MeshNormalFilter.h:345` |
| Vertex projection | target plane projection, with line projection for flipped faces | `src/MeshNormalFilter.h:493` |
| Update convergence | `sqrt(mean(|v_new-v_prev|^2)) <= meshUpdateDisplacementEps` | `src/MeshNormalFilter.h:549` |

## Target File Map

Modify:

- `Package.swift`: add `MeshDenoiserNative`, add `MeshDenoiserBench`, and make tests depend on the native target.
- `Sources/MeshDenoiserKit/MeshDenoiseParameters.swift`: add `Backend` while preserving current default behavior until final cutover.
- `Sources/MeshDenoiserKit/MeshDenoiseError.swift`: add native-specific surfaced errors that fit the current error model.
- `Sources/MeshDenoiserKit/MeshDenoiser.swift`: route `.reference`, `.nativeCPU`, `.nativeGPU`, and `.automatic`.
- `Tests/MeshDenoiserKitTests/DenoiseContractTests.swift`: run contract tests against supported backends.
- `Tests/MeshDenoiserKitTests/GoldenParityTests.swift`: pin current exact golden parity to `.reference`; add native tolerance parity separately.
- `README.md`: document native backend maturity, backend selection, and benchmark command after implementation.

Create:

- `Sources/MeshDenoiserNative/NativeDenoiser.swift`: orchestrator and public native entry point.
- `Sources/MeshDenoiserNative/NativeTypes.swift`: parameters, errors, topology/geometry records, CSR helpers, backend metrics.
- `Sources/MeshDenoiserNative/MeshValidation.swift`: raw buffer validation without OpenMesh.
- `Sources/MeshDenoiserNative/MeshNormalization.swift`: normalize/restore contract.
- `Sources/MeshDenoiserNative/MeshConnectivity.swift`: normals, areas, centroids, edges, adjacency.
- `Sources/MeshDenoiserNative/GuidanceNormals.swift`: patch-guidance implementation.
- `Sources/MeshDenoiserNative/NeighborSearch.swift`: uniform-grid face-neighbor search.
- `Sources/MeshDenoiserNative/FilterPrecompute.swift`: static weights, lambda rescale, face-neighbor CSR.
- `Sources/MeshDenoiserNative/FilterCPU.swift`: deterministic CPU fixed-point filter.
- `Sources/MeshDenoiserNative/FilterGPU.swift`: Metal context, buffers, dispatch, GPU fallback handling.
- `Sources/MeshDenoiserNative/FilterKernels.metal`: dynamic-weight, gather-normalize, convergence-reduction kernels.
- `Sources/MeshDenoiserNative/VertexUpdate.swift`: sparse matrix-equivalent operator, projection iterations, matrix-free SPD CG solve.
- `Sources/MeshDenoiserNative/NativeBenchmarkSupport.swift`: package benchmark hooks for isolated native stage measurements.
- `Sources/MeshDenoiserNative/NativeMetrics.swift`: optional per-stage timing for tests and benchmark output.
- `Sources/MeshDenoiserBench/main.swift`: macOS benchmark executable.
- `Tests/MeshDenoiserKitTests/NativeFixtures.swift`: shared fixture loading and backend helpers.
- `Tests/MeshDenoiserKitTests/NativeValidationTests.swift`.
- `Tests/MeshDenoiserKitTests/MeshConnectivityTests.swift`.
- `Tests/MeshDenoiserKitTests/GuidanceNormalsTests.swift`.
- `Tests/MeshDenoiserKitTests/NeighborSearchTests.swift`.
- `Tests/MeshDenoiserKitTests/FilterPrecomputeTests.swift`.
- `Tests/MeshDenoiserKitTests/FilterCPUTests.swift`.
- `Tests/MeshDenoiserKitTests/VertexUpdateTests.swift`.
- `Tests/MeshDenoiserKitTests/NativeParityTests.swift`.
- `Tests/MeshDenoiserKitTests/GPUParityTests.swift`.

## Data Model

Create compact value types that avoid OpenMesh-style object graphs:

```swift
public struct NativeDenoiseParameters: Sendable, Equatable {
    public var lambda: Double = 0.15
    public var eta: Double = 2.2
    public var mu: Double = 0.2
    public var nu: Double = 0.25
    public var meshUpdateClosenessWeight: Double = 0.001
    public var meshUpdateIterations: Int = 20
    public var meshUpdateDisplacementEps: Double = 0.1
    public var outerIterations: Int = 1
    public init() {}
}

struct NativeMesh: Sendable {
    var positions: [SIMD3<Float>]
    var triangles: [SIMD3<UInt32>]
}

struct FaceGeometry: Sendable {
    var normals: [SIMD3<Float>]
    var centroids: [SIMD3<Float>]
    var areaWeights: [Float]
}

struct Edge: Hashable, Sendable {
    var a: UInt32
    var b: UInt32
}

struct EdgeRecord: Sendable {
    var vertices: Edge
    var face0: UInt32
    var face1: UInt32?
}

struct CSRRows<Value: Sendable>: Sendable {
    var offsets: [UInt32]
    var values: [Value]

    func range(for row: Int) -> Range<Int> {
        Int(offsets[row])..<Int(offsets[row + 1])
    }
}
```

Keep all Metal-facing per-face vectors as `SIMD4<Float>` buffers for alignment, with `.w = 0`.

## Task 1: Baseline Gates And Backend Skeleton

**Files:**

- Modify: `Package.swift`
- Create: `Sources/MeshDenoiserNative/NativeTypes.swift`
- Create: `Sources/MeshDenoiserNative/NativeDenoiser.swift`
- Modify: `Sources/MeshDenoiserKit/MeshDenoiseParameters.swift`
- Modify: `Sources/MeshDenoiserKit/MeshDenoiseError.swift`
- Modify: `Sources/MeshDenoiserKit/MeshDenoiser.swift`
- Create: `Tests/MeshDenoiserKitTests/NativeValidationTests.swift`

- [ ] Add the native target to `Package.swift`:

```swift
.target(
    name: "MeshDenoiserNative",
    path: "Sources/MeshDenoiserNative"
)
```

- [ ] Add `MeshDenoiserNative` to the `MeshDenoiserKit` target dependencies and to `MeshDenoiserKitTests` dependencies.

- [ ] Add `MeshDenoiseParameters.Backend`:

```swift
public enum Backend: Sendable, CaseIterable {
    case automatic
    case nativeGPU
    case nativeCPU
    case reference
}

public var backend: Backend = .reference
```

Keep `.reference` as the default in this task so every existing test remains exact.

- [ ] Add a stub native entry point:

```swift
public enum NativeDenoiser {
    public static var isGPUAvailable: Bool {
        MTLCreateSystemDefaultDevice() != nil
    }

    public static func denoise(
        positions: [SIMD3<Float>],
        indices: [UInt32],
        parameters: NativeDenoiseParameters,
        useGPU: Bool,
        progress: (@Sendable (Int, Int) -> Bool)?
    ) throws -> [SIMD3<Float>] {
        throw NativeDenoiseError.notImplemented
    }
}
```

- [ ] Route `.reference` to the current C bridge and route native cases to `NativeDenoiser`, mapping native errors to the existing public error enum. `nativeGPU` must throw `.gpuUnavailable` when no Metal device exists. `.automatic` must prefer GPU when available and otherwise run CPU.

- [ ] Add tests that verify backend routing does not change current behavior:

```swift
func testDefaultBackendIsReference() {
    XCTAssertEqual(MeshDenoiseParameters().backend, .reference)
}

func testNativeCPUCurrentlyThrowsSolverFailedFromPublicAPI() async {
    let mesh = DenoiseContractTests.noisyOctahedron()
    var params = MeshDenoiseParameters()
    params.backend = .nativeCPU
    await assertThrows(MeshDenoiseError.solverFailed) {
        _ = try await MeshDenoiser.denoise(
            positions: mesh.positions,
            indices: mesh.indices,
            parameters: params
        )
    }
}
```

- [ ] Run:

```bash
swift test
```

Expected: existing tests pass; new native stub test passes because the public API maps `.notImplemented` to `.solverFailed`.

## Task 2: Native Validation And Normalize/Restore

**Files:**

- Create: `Sources/MeshDenoiserNative/MeshValidation.swift`
- Create: `Sources/MeshDenoiserNative/MeshNormalization.swift`
- Modify: `Sources/MeshDenoiserNative/NativeDenoiser.swift`
- Create: `Tests/MeshDenoiserKitTests/NativeValidationTests.swift`

- [ ] Implement validation with the same observable contract as the C bridge:

```swift
enum MeshValidation {
    static func makeMesh(positions: [SIMD3<Float>], indices: [UInt32]) throws -> NativeMesh {
        guard !positions.isEmpty, !indices.isEmpty, indices.count.isMultiple(of: 3) else {
            throw NativeDenoiseError.invalidInput
        }
        for p in positions where !p.x.isFinite || !p.y.isFinite || !p.z.isFinite {
            throw NativeDenoiseError.invalidInput
        }
        var triangles = [SIMD3<UInt32>]()
        triangles.reserveCapacity(indices.count / 3)
        let vertexCount = UInt32(positions.count)
        for offset in stride(from: 0, to: indices.count, by: 3) {
            let tri = SIMD3(indices[offset], indices[offset + 1], indices[offset + 2])
            guard tri.x < vertexCount, tri.y < vertexCount, tri.z < vertexCount else {
                throw NativeDenoiseError.invalidInput
            }
            guard tri.x != tri.y, tri.y != tri.z, tri.x != tri.z else {
                throw NativeDenoiseError.invalidInput
            }
            triangles.append(tri)
        }
        return NativeMesh(positions: positions, triangles: triangles)
    }
}
```

- [ ] Implement parameter validation:

```swift
extension NativeDenoiseParameters {
    var isValid: Bool {
        lambda > 0 && eta > 0 && mu > 0 && nu > 0
            && meshUpdateClosenessWeight >= 0
            && meshUpdateIterations > 0
            && meshUpdateDisplacementEps >= 0
            && outerIterations > 0
    }
}
```

- [ ] Implement normalization:

```swift
struct MeshNormalization: Sendable {
    var originalCenter: SIMD3<Double>
    var originalScale: Double

    static func normalize(_ positions: [SIMD3<Float>]) throws -> (positions: [SIMD3<Float>], transform: MeshNormalization) {
        let center = positions.reduce(SIMD3<Double>(repeating: 0)) {
            $0 + SIMD3<Double>(Double($1.x), Double($1.y), Double($1.z))
        } / Double(positions.count)
        var minP = SIMD3<Double>(repeating: .infinity)
        var maxP = SIMD3<Double>(repeating: -.infinity)
        for p in positions {
            let d = SIMD3<Double>(Double(p.x), Double(p.y), Double(p.z))
            minP = min(minP, d)
            maxP = max(maxP, d)
        }
        let scale = length(maxP - minP)
        guard scale.isFinite, scale > 0 else { throw NativeDenoiseError.invalidInput }
        let normalized = positions.map { p -> SIMD3<Float> in
            let d = (SIMD3<Double>(Double(p.x), Double(p.y), Double(p.z)) - center) / scale
            return SIMD3<Float>(Float(d.x), Float(d.y), Float(d.z))
        }
        return (normalized, MeshNormalization(originalCenter: center, originalScale: scale))
    }

    func restore(_ positions: [SIMD3<Float>]) -> [SIMD3<Float>] {
        let currentCenter = positions.reduce(SIMD3<Double>(repeating: 0)) {
            $0 + SIMD3<Double>(Double($1.x), Double($1.y), Double($1.z))
        } / Double(positions.count)
        return positions.map { p in
            let d = (SIMD3<Double>(Double(p.x), Double(p.y), Double(p.z)) - currentCenter)
                * originalScale + originalCenter
            return SIMD3<Float>(Float(d.x), Float(d.y), Float(d.z))
        }
    }
}
```

- [ ] Test invalid counts, NaN, out-of-range index, degenerate triangle, zero-scale mesh, and round-trip normalize/restore within `1e-6`.

- [ ] Run:

```bash
swift test --filter NativeValidationTests
```

## Task 3: Connectivity And Geometry

**Files:**

- Create: `Sources/MeshDenoiserNative/MeshConnectivity.swift`
- Create: `Tests/MeshDenoiserKitTests/MeshConnectivityTests.swift`

- [ ] Implement face geometry:

```swift
enum MeshConnectivity {
    static func faceGeometry(mesh: NativeMesh) throws -> FaceGeometry {
        var normals = [SIMD3<Float>]()
        var centroids = [SIMD3<Float>]()
        var rawAreas = [Float]()
        normals.reserveCapacity(mesh.triangles.count)
        centroids.reserveCapacity(mesh.triangles.count)
        rawAreas.reserveCapacity(mesh.triangles.count)

        for tri in mesh.triangles {
            let p0 = mesh.positions[Int(tri.x)]
            let p1 = mesh.positions[Int(tri.y)]
            let p2 = mesh.positions[Int(tri.z)]
            let crossValue = cross(p1 - p0, p2 - p0)
            let doubleArea = length(crossValue)
            guard doubleArea.isFinite, doubleArea > 0 else { throw NativeDenoiseError.invalidInput }
            normals.append(crossValue / doubleArea)
            centroids.append((p0 + p1 + p2) / 3)
            rawAreas.append(doubleArea * 0.5)
        }

        let meanArea = rawAreas.reduce(0, +) / Float(rawAreas.count)
        guard meanArea.isFinite, meanArea > 0 else { throw NativeDenoiseError.invalidInput }
        return FaceGeometry(
            normals: normals,
            centroids: centroids,
            areaWeights: rawAreas.map { $0 / meanArea }
        )
    }
}
```

- [ ] Implement edge records using sorted undirected edge keys. Reject non-manifold edges with more than two adjacent faces.

- [ ] Implement vertex-to-face CSR and vertex-to-nonboundary-edge CSR from the edge records.

- [ ] Implement `averageNeighborFaceCentroidDistance` over non-boundary edges only. Throw `.invalidInput` if there are no interior edges.

- [ ] Test:

```swift
func testOctahedronConnectivityCounts() throws {
    let meshData = DenoiseContractTests.noisyOctahedron()
    let mesh = try MeshValidation.makeMesh(positions: meshData.positions, indices: meshData.indices)
    let connectivity = try MeshConnectivity.build(mesh: mesh)
    XCTAssertEqual(connectivity.faceGeometry.normals.count, 8)
    XCTAssertEqual(connectivity.edges.count, 12)
    XCTAssertEqual(connectivity.vertexToFaces.offsets.count, mesh.positions.count + 1)
    XCTAssertTrue(connectivity.averageNeighborFaceCentroidDistance > 0)
}
```

- [ ] Run:

```bash
swift test --filter MeshConnectivityTests
```

## Task 4: Guidance Normals

**Files:**

- Create: `Sources/MeshDenoiserNative/GuidanceNormals.swift`
- Create: `Tests/MeshDenoiserKitTests/GuidanceNormalsTests.swift`

- [ ] Port the C++ patch-guidance behavior directly:

```swift
enum GuidanceNormals {
    static func compute(connectivity: MeshConnectivity.Result) -> [SIMD3<Float>] {
        let faceCount = connectivity.faceGeometry.normals.count
        var edgeSaliency = [Float](repeating: 0, count: connectivity.edges.count)
        for (edgeIndex, edge) in connectivity.edges.enumerated() {
            guard let face1 = edge.face1 else { continue }
            let n0 = connectivity.faceGeometry.normals[Int(edge.face0)]
            let n1 = connectivity.faceGeometry.normals[Int(face1)]
            edgeSaliency[edgeIndex] = length(n0 - n1)
        }

        var patchConsistency = [Float](repeating: 0, count: faceCount)
        var patchAverage = [SIMD3<Float>](repeating: .zero, count: faceCount)
        var patchFaces = [[UInt32]]()
        patchFaces.reserveCapacity(faceCount)

        for faceIndex in 0..<faceCount {
            let patch = collectPatchFaces(faceIndex: faceIndex, connectivity: connectivity)
            let patchEdges = collectPatchEdges(faceIndex: faceIndex, connectivity: connectivity)
            patchFaces.append(patch)

            let sumSaliency = patchEdges.reduce(Float(0)) { $0 + edgeSaliency[Int($1)] }
            let maxSaliency = patchEdges.map { edgeSaliency[Int($0)] }.max() ?? 0
            var maxNormalDiff: Float = 0
            for i in 0..<patch.count {
                for j in (i + 1)..<patch.count {
                    let diff = length(connectivity.faceGeometry.normals[Int(patch[i])] -
                                      connectivity.faceGeometry.normals[Int(patch[j])])
                    maxNormalDiff = max(maxNormalDiff, diff)
                }
            }
            patchConsistency[faceIndex] = (maxSaliency / (1e-9 + sumSaliency)) * maxNormalDiff

            var average = SIMD3<Float>.zero
            for patchFace in patch {
                average += connectivity.faceGeometry.normals[Int(patchFace)]
                    * connectivity.faceGeometry.areaWeights[Int(patchFace)]
            }
            patchAverage[faceIndex] = normalize(average)
        }

        return (0..<faceCount).map { faceIndex in
            let candidates = patchFaces[faceIndex]
            let best = candidates.min { patchConsistency[Int($0)] < patchConsistency[Int($1)] }!
            return patchAverage[Int(best)]
        }
    }
}
```

- [ ] Use stamped scratch arrays instead of `Set` in `collectPatchFaces` and `collectPatchEdges` to keep memory predictable on large meshes.

- [ ] Test flat planar meshes return the original normals and the noisy octahedron returns one unit-length guidance normal per face.

- [ ] Run:

```bash
swift test --filter GuidanceNormalsTests
```

## Task 5: Neighbor Search And Static Filter Precompute

**Files:**

- Create: `Sources/MeshDenoiserNative/NeighborSearch.swift`
- Create: `Sources/MeshDenoiserNative/FilterPrecompute.swift`
- Create: `Tests/MeshDenoiserKitTests/NeighborSearchTests.swift`
- Create: `Tests/MeshDenoiserKitTests/FilterPrecomputeTests.swift`

- [ ] Implement uniform-grid neighbor search equivalent to the current C++ path: grid cell width is `radius`, search the 27 adjacent cells, emit pairs only when `j > i`, and accept strict `distanceSquared < radiusSquared`.

- [ ] Store pair data compactly:

```swift
struct NeighborPair: Sendable, Equatable {
    var face0: UInt32
    var face1: UInt32
    var distance: Float
}

struct FilterPrecompute: Sendable {
    var pairs: [NeighborPair]
    var staticWeights: [Float]
    var faceNeighborRows: CSRRows<FaceNeighbor>
    var rescaledLambda: Float
    var weightedInitialSignals: [SIMD3<Float>]
    var convergenceThreshold: Float
}

struct FaceNeighbor: Sendable, Equatable {
    var neighborFace: UInt32
    var pairIndex: UInt32
}
```

- [ ] Implement weight precompute:

```swift
let hSpatial = Float(-0.5 / (etaPrime * etaPrime))
let hGuidance = Float(-0.5 / (parameters.mu * parameters.mu))
let areaSpatial = (area0 + area1) * exp(hSpatial * distance * distance)
let staticWeight = (area0 + area1) * exp(
    hGuidance * length_squared(guidance0 - guidance1) + hSpatial * distance * distance
)
```

- [ ] Rescale lambda using the area-spatial sum, then compute:

```swift
let scale = Float(2 * parameters.nu * parameters.nu) / rescaledLambda
weightedInitialSignals[i] = initialNormals[i] * areaWeights[i] * scale
```

- [ ] Build `faceNeighborRows` using degree counting and cursor fill, matching the C++ CSR-style layout.

- [ ] Test neighbor search against a brute-force implementation on the fixture and assert pair sets match exactly.

- [ ] Test static weights against a direct scalar calculation on two known pairs with tolerance `1e-6`.

- [ ] Run:

```bash
swift test --filter NeighborSearchTests
swift test --filter FilterPrecomputeTests
```

## Task 6: CPU Fixed-Point Filter

**Files:**

- Create: `Sources/MeshDenoiserNative/FilterCPU.swift`
- Create: `Tests/MeshDenoiserKitTests/FilterCPUTests.swift`

- [ ] Implement the CPU filter as the parity baseline:

```swift
enum FilterCPU {
    static func run(
        initialSignals: [SIMD3<Float>],
        areaWeights: [Float],
        precompute: FilterPrecompute,
        nu: Float,
        maxIterations: Int = 100,
        avgDispEps: Float = 2 * sin(0.1 * .pi / 180)
    ) -> (signals: [SIMD3<Float>], iterations: Int, converged: Bool) {
        var signals = initialSignals
        var previous = signals
        var dynamicWeights = [Float](repeating: 0, count: precompute.pairs.count)
        var filtered = precompute.weightedInitialSignals
        let h = Float(-0.5) / (nu * nu)
        let threshold = areaWeights.reduce(0, +) * avgDispEps * avgDispEps

        for iteration in 1...maxIterations {
            previous = signals
            for (pairIndex, pair) in precompute.pairs.enumerated() {
                let diff = signals[Int(pair.face0)] - signals[Int(pair.face1)]
                dynamicWeights[pairIndex] = precompute.staticWeights[pairIndex] * exp(h * length_squared(diff))
            }
            filtered = precompute.weightedInitialSignals
            for faceIndex in filtered.indices {
                for valueIndex in precompute.faceNeighborRows.range(for: faceIndex) {
                    let item = precompute.faceNeighborRows.values[valueIndex]
                    filtered[faceIndex] += signals[Int(item.neighborFace)] * dynamicWeights[Int(item.pairIndex)]
                }
                filtered[faceIndex] = normalize(filtered[faceIndex])
            }
            signals = filtered

            var displacement: Float = 0
            for index in signals.indices {
                displacement += areaWeights[index] * length_squared(signals[index] - previous[index])
            }
            if displacement <= threshold {
                return (signals, iteration, true)
            }
        }
        return (signals, maxIterations, false)
    }
}
```

- [ ] Add fixture tests that compare CPU filtered normals to the C++ reference backend after one outer iteration at end-to-end tolerance, using the final `NativeDenoiser` path once Task 8 connects it.

- [ ] Add a deterministic small test for convergence where initial signals already equal the weighted solution direction.

- [ ] Run:

```bash
swift test --filter FilterCPUTests
```

## Task 7: Vertex Update

**Files:**

- Create: `Sources/MeshDenoiserNative/VertexUpdate.swift`
- Create: `Tests/MeshDenoiserKitTests/VertexUpdateTests.swift`
- Create: `Sources/MeshDenoiserNative/NativeBenchmarkSupport.swift`

- [x] Build the sparse matrix-equivalent system for `M = A^T A + wI`. For each face with vertices `(a,b,c)`, add the `I - J/3` block:

```swift
let diagonal = 2.0 / 3.0
let offDiagonal = -1.0 / 3.0
```

Each triangle contributes these values to the 3-by-3 vertex block for each coordinate solve. Add `w` to every diagonal.

- [x] Use an isolated matrix-free SPD conjugate-gradient solver for the sparse vertex update operator:

```swift
let ap = applySystem(vector: p, triangles: triangles, closenessWeight: closenessWeight)
let alpha = rsOld / dot(p, ap)
```

The solver uses the same operator and RHS contract as a direct sparse solve would use, so an Accelerate implementation can still replace `solveSystem(...)` without changing projection logic or public API.

- [ ] Deferred optimization candidate: replace CG with Accelerate sparse direct factorization only if larger real-world fixtures show vertex update is the dominant runtime:

```swift
import Accelerate

// Build SparseMatrix_Double in compressed row or compressed column form,
// then factor with:
let factor = SparseFactor(SparseFactorizationCholesky, matrix)
// Solve three RHS columns with SparseSolve.
```

If Cholesky fails for a valid mesh, retry the SDK's available LDLT variant before surfacing `.solverFailed`.

- [x] Add isolated vertex-update benchmark support:

```bash
swift run -c release MeshDenoiserBench --mode vertex-update --faces 81920
```

Measured with deterministic generated meshes and deterministic target-normal perturbations:

```text
mode,vertices,faces,total_secs,peak_memory_mb,checksum
vertex-update,10002,20000,0.040062875,13.25,7.243371998033599e-08
vertex-update,40806,81608,0.240081542,33.703125,-1.055188844995003e-06
vertex-update,163622,327240,1.177580917,103.03125,6.254467636440664e-07
```

Decision: keep the current CG implementation for this port slice. At 328k faces, isolated vertex update is `1.177580917s` versus the latest full native GPU run at `4.399863958s` in the all-backend matrix and `4.92941075s` in single-backend confirmation. The direct Accelerate path would add sparse assembly/factorization complexity and likely more peak memory; defer it until real app fixtures prove the solver dominates.

- [x] Implement per-iteration `B` generation exactly:

```swift
if dot(currentNormal, targetNormal) >= 0 {
    targetPosition = centeredPosition - targetNormal * dot(targetNormal, centeredPosition)
} else {
    targetPosition = projectToPrincipalLineInTargetPlane(centeredTriangle, targetNormal)
}
```

Use a small closed-form orthonormal basis for the target plane instead of a general SVD for the normal vector. For the principal line in 2D, compute the dominant eigenvector of the 2-by-2 covariance matrix; this matches the SVD result without pulling in a dense linear algebra dependency.

- [x] Build RHS as `w * X0 + A^T * B` and solve X, Y, Z as three columns.

- [x] Stop when RMS displacement is at or below `meshUpdateDisplacementEps`; if the threshold is zero, always run `meshUpdateIterations`.

- [x] Test the assembled matrix on a single triangle:

```swift
XCTAssertEqual(block[0,0], 2.0 / 3.0 + w, accuracy: 1e-12)
XCTAssertEqual(block[0,1], -1.0 / 3.0, accuracy: 1e-12)
```

- [x] Test plane projection preserves triangle centroid in centered coordinates.

- [x] Test one complete update pass on the octahedron produces finite positions and preserves vertex count/order.

- [x] Run:

```bash
swift test --filter VertexUpdateTests
```

## Task 8: End-To-End Native CPU Backend

**Files:**

- Modify: `Sources/MeshDenoiserNative/NativeDenoiser.swift`
- Create: `Tests/MeshDenoiserKitTests/NativeParityTests.swift`
- Modify: `Tests/MeshDenoiserKitTests/DenoiseContractTests.swift`

- [x] Connect the native CPU pipeline:

```swift
public static func denoise(
    positions: [SIMD3<Float>],
    indices: [UInt32],
    parameters: NativeDenoiseParameters,
    useGPU: Bool,
    progress: (@Sendable (Int, Int) -> Bool)?
) throws -> [SIMD3<Float>] {
    guard parameters.isValid else { throw NativeDenoiseError.invalidParameters }
    var mesh = try MeshValidation.makeMesh(positions: positions, indices: indices)
    let normalized = try MeshNormalization.normalize(mesh.positions)
    mesh.positions = normalized.positions

    for outer in 0..<parameters.outerIterations {
        let connectivity = try MeshConnectivity.build(mesh: mesh)
        let guidance = GuidanceNormals.compute(connectivity: connectivity)
        let etaPrime = Float(parameters.eta) * connectivity.averageNeighborFaceCentroidDistance
        let pairs = try NeighborSearch.findPairs(centroids: connectivity.faceGeometry.centroids, radius: 3 * etaPrime)
        let precompute = try FilterPrecompute.build(
            pairs: pairs,
            guidanceNormals: guidance,
            initialNormals: connectivity.faceGeometry.normals,
            areaWeights: connectivity.faceGeometry.areaWeights,
            etaPrime: etaPrime,
            parameters: parameters
        )
        let filteredNormals = FilterCPU.run(
            initialSignals: connectivity.faceGeometry.normals,
            areaWeights: connectivity.faceGeometry.areaWeights,
            precompute: precompute,
            nu: Float(parameters.nu)
        ).signals
        mesh.positions = try VertexUpdate.run(
            mesh: mesh,
            targetNormals: filteredNormals,
            parameters: parameters
        )
        guard progress?(outer + 1, parameters.outerIterations) ?? true else {
            throw NativeDenoiseError.cancelled
        }
    }

    return normalized.transform.restore(mesh.positions)
}
```

- [x] Add native CPU contract tests for vertex count/order, finite positions, invalid input, invalid parameters, cancellation, and progress.

- [x] Add tolerance parity against `.reference` on `noisy_icosphere.obj`:

```swift
XCTAssertLessThan(maxError, 2e-3)
XCTAssertLessThan(meanError, 2e-4)
```

Record measured max/mean values in the test failure message so the threshold can be tightened with evidence after first implementation.

- [x] Run:

```bash
swift test --filter NativeParityTests
swift test --filter DenoiseContractTests
```

## Task 9: Metal Fixed-Point Filter

**Files:**

- Create: `Sources/MeshDenoiserNative/FilterGPU.swift`
- Create: `Sources/MeshDenoiserNative/FilterKernels.metal`
- Create: `Tests/MeshDenoiserKitTests/GPUParityTests.swift`
- Modify: `Sources/MeshDenoiserNative/NativeDenoiser.swift`

- [x] Keep preprocessing and vertex update on CPU. Move only fixed-point filter work to Metal.

- [x] Use the full planned buffer set:

| Buffer | Element | Direction |
|---|---|---|
| `signalsA`, `signalsB` | `SIMD4<Float>` | ping-pong |
| `weightedInitialSignals` | `SIMD4<Float>` | read |
| `staticWeights` | `float` | read |
| `rowOffsets` | `uint` | read |
| `rowValues` | `{ uint neighborFace; uint pairIndex; }` | read |
| `areaWeights` | `float` | read |
| `partialDisplacement` | `float` | write/readback |

- [x] Implement dynamic-weight computation. The original separate kernel was implemented first, then superseded by fused gather-side computation to remove the `pairs` and `dynamicWeights` Metal buffers from the active runtime.

- [x] Implement fused gather and normalize:

```metal
kernel void gather_and_normalize(
    device const float4 *signalsIn [[buffer(0)]],
    device const float4 *weightedInitial [[buffer(1)]],
    device const uint *rowOffsets [[buffer(2)]],
    device const FaceNeighborGPU *rowValues [[buffer(3)]],
    device const float *staticWeights [[buffer(4)]],
    device float4 *signalsOut [[buffer(5)]],
    constant float &hDynamic [[buffer(6)]],
    uint face [[thread_position_in_grid]]
)
```

- [x] Implement kernel 3 for area-weighted displacement with block-level partial sums. Read back partial sums once per fixed-point iteration. Keep the CPU convergence decision; command-buffer latency is acceptable because convergence is checked once per iteration and the CPU fallback remains available.

- [x] Use `maxTotalThreadsPerThreadgroup` to choose threadgroup size. Current implementation caps at 256 only when the pipeline limit allows it; `threadExecutionWidth` tuning remains a benchmark follow-up.

- [x] Add GPU-vs-CPU filter parity on the fixture:

```swift
XCTAssertLessThan(maxNormalError, 1e-4)
```

- [x] Add end-to-end `.nativeGPU` parity against `.nativeCPU` with vertex max error below `2e-3` on the current fixture.

- [x] Run on a Metal-capable machine:

```bash
swift test --filter GPUParityTests
```

If no Metal device exists, tests must skip with `XCTSkip`.

## Task 10: Benchmarks And Default Backend Cutover

**Files:**

- Create: `Sources/MeshDenoiserBench/main.swift`
- Modify: `Package.swift`
- Modify: `Sources/MeshDenoiserKit/MeshDenoiseParameters.swift`
- Modify: `README.md`

- [x] Add an executable target:

```swift
.executableTarget(
    name: "MeshDenoiserBench",
    dependencies: ["MeshDenoiserKit", "MeshDenoiserNative"],
    path: "Sources/MeshDenoiserBench"
)
```

- [x] Benchmark `.reference`, `.nativeCPU`, and `.nativeGPU` on deterministic generated fixtures at approximately 20k, 82k, and 328k faces, and on file-backed USD/USDZ/OBJ meshes with `--input`. Print CSV:

```text
backend,vertices,faces,total_secs,max_error_vs_reference,mean_error_vs_reference,peak_memory_mb
```

Measured on Apple Silicon with `swift run -c release MeshDenoiserBench --faces <N> --backend all`.
`--backend all` runs each backend in a child process so peak RSS is per backend.
For app-style validation, use `swift run -c release MeshDenoiserBench --input path/to/model.usdz --backend all`.
For production fixture sweeps, use `swift run -c release MeshDenoiserBench --input-dir path/to/usdz-fixtures --backend all` or `swift run -c release MeshDenoiserBench --input-list fixtures.txt --backend all`.
Use `--keep-going` on broad USDZ sweeps when the fixture folder may contain invalid, open, or non-triangular assets; successful rows add `status=ok` and failures add `status=failed,error=<message>`. When the file can still be loaded, failure messages include mesh diagnostics with vertex count, face count, boundary edge count, non-manifold edge count, and degenerate face count.
Add `--fail-on-error` for CI fixture gates; the command still writes every row, prints a batch summary to stderr, and exits `1` if any row failed.
Use `--max-error-threshold 0.002 --mean-error-threshold 0.0002` to enforce the current native/reference release tolerance gate on production USDZ fixtures. Use `--max-total-seconds-threshold` and `--max-memory-mb-threshold` on the same sweep to enforce runtime and peak-RSS budgets.
The benchmark importer uses ModelIO on macOS for `.usd`, `.usda`, `.usdc`, and `.usdz`, flattens mesh transforms, and triangulates supported submeshes before calling `MeshDenoiserKit`.

20k target:

```text
backend,vertices,faces,total_secs,max_error_vs_reference,mean_error_vs_reference,peak_memory_mb
reference,10002,20000,0.165296,0.0,0.0,203.15625
nativeCPU,10002,20000,0.153562417,8.763125151745044e-06,4.651481790922126e-06,130.15625
nativeGPU,10002,20000,0.148916167,8.7058015196817e-06,4.659569080285767e-06,172.515625
```

82k target:

```text
backend,vertices,faces,total_secs,max_error_vs_reference,mean_error_vs_reference,peak_memory_mb
reference,40806,81608,0.9532565,0.0,0.0,1058.8125
nativeCPU,40806,81608,0.856448667,4.158279625698924e-05,1.0190995399195528e-05,747.1875
nativeGPU,40806,81608,0.672686625,4.158279625698924e-05,1.0190980173210907e-05,967.25
```

328k target:

```text
backend,vertices,faces,total_secs,max_error_vs_reference,mean_error_vs_reference,peak_memory_mb
reference,163622,327240,7.630192666,0.0,0.0,7658.296875
nativeCPU,163622,327240,6.022254125,0.00020745472284033895,3.6978420425139504e-05,5608.921875
nativeGPU,163622,327240,4.399863958,0.00020745472284033895,3.6978420805664054e-05,7242.40625
```

328k native GPU single-backend confirmation after fused gather:

```text
backend,vertices,faces,total_secs,max_error_vs_reference,mean_error_vs_reference,peak_memory_mb
nativeGPU,163622,327240,4.92941075,0.00020745472284033895,3.6978420805664054e-05,6358.0625
```

Checked-in small OBJ fixture:

```text
backend,vertices,faces,total_secs,max_error_vs_reference,mean_error_vs_reference,peak_memory_mb
reference,162,320,0.001983833,0.0,0.0,9.546875
nativeCPU,162,320,0.00137525,1.884864389012364e-07,6.607910615393828e-08,8.40625
nativeGPU,162,320,0.029900666,1.884864389012364e-07,6.676634699704701e-08,15.734375
```

Local untracked `out.obj` file-backed mesh (`83,255` vertices, `150,000` faces):

```text
backend,vertices,faces,total_secs,max_error_vs_reference,mean_error_vs_reference,peak_memory_mb
reference,83255,150000,1.069218834,0.0,0.0,887.734375
nativeCPU,83255,150000,1.227231959,1.313160373683786e-05,2.2525185775428545e-06,541.703125
nativeGPU,83255,150000,0.910974208,1.313160373683786e-05,2.2525154695869385e-06,700.734375
```

Local untracked `out.obj` isolated vertex-update measurement:

```text
mode,vertices,faces,total_secs,peak_memory_mb,checksum
vertex-update,83255,150000,0.456165958,71.828125,3.121281195866965e-07
```

USDZ smoke fixture from local `tinyusdz` dependency cache (`build/_deps/tinyusdz-src/models/cube.usdz`):

```text
backend,vertices,faces,total_secs,max_error_vs_reference,mean_error_vs_reference,peak_memory_mb
reference,24,12,0.000213917,0.0,0.0,19.65625
nativeCPU,24,12,0.00020075,7.62939453125e-06,7.62939453125e-06,19.671875
nativeGPU,24,12,0.0296935,7.62939453125e-06,7.62939453125e-06,25.34375
```

USDC smoke fixture from local `tinyusdz` dependency cache (`build/_deps/tinyusdz-src/models/suzanne.usdc`):

```text
backend,vertices,faces,total_secs,max_error_vs_reference,mean_error_vs_reference,peak_memory_mb
reference,1968,968,0.004838041,0.0,0.0,25.03125
nativeCPU,1968,968,0.003970292,1.341104507446289e-07,3.2913903840671047e-08,22.96875
nativeGPU,1968,968,0.0316405,1.341104507446289e-07,3.314860653697332e-08,29.96875
```

Note: the USD/USDZ cache fixtures are loader smoke tests, not representative performance assets. The next cutoff decision should use committed production-like USDZ fixtures.

Batch USD/USDZ validation smoke via `--input-list` over local `cube.usdz` and `suzanne.usdc`:

```text
input,backend,vertices,faces,total_secs,max_error_vs_reference,mean_error_vs_reference,peak_memory_mb
/Users/markus/GIT/meshdenoiser/build/_deps/tinyusdz-src/models/cube.usdz,reference,24,12,0.000371666,0.0,0.0,19.78125
/Users/markus/GIT/meshdenoiser/build/_deps/tinyusdz-src/models/cube.usdz,nativeCPU,24,12,0.000246333,7.62939453125e-06,7.62939453125e-06,19.65625
/Users/markus/GIT/meshdenoiser/build/_deps/tinyusdz-src/models/cube.usdz,nativeGPU,24,12,0.027932666,7.62939453125e-06,7.62939453125e-06,25.140625
/Users/markus/GIT/meshdenoiser/build/_deps/tinyusdz-src/models/suzanne.usdc,reference,1968,968,0.004559959,0.0,0.0,25.0
/Users/markus/GIT/meshdenoiser/build/_deps/tinyusdz-src/models/suzanne.usdc,nativeCPU,1968,968,0.003720167,1.341104507446289e-07,3.2913903840671047e-08,23.4375
/Users/markus/GIT/meshdenoiser/build/_deps/tinyusdz-src/models/suzanne.usdc,nativeGPU,1968,968,0.029697959,1.341104507446289e-07,3.314860653697332e-08,29.5
```

Batch directory discovery smoke via `--input-dir` over symlinked local `cube.usdz` and `suzanne.usdc` passed with the reference backend. ModelIO emitted its existing USD metadata warning on `suzanne.usdc`, but loading and denoising completed.

Failure-tolerant batch smoke via `--keep-going` over valid `cube.usdz` and flat/non-denoiser-ready `texture-cat-plane.usdz`:

```text
input,backend,vertices,faces,total_secs,max_error_vs_reference,mean_error_vs_reference,peak_memory_mb,status,error
/Users/markus/GIT/meshdenoiser/build/_deps/tinyusdz-src/models/cube.usdz,reference,24,12,0.000306875,0.0,0.0,19.859375,ok,
/Users/markus/GIT/meshdenoiser/build/_deps/tinyusdz-src/models/cube.usdz,nativeCPU,24,12,0.0002535,7.62939453125e-06,7.62939453125e-06,19.71875,ok,
/Users/markus/GIT/meshdenoiser/build/_deps/tinyusdz-src/models/cube.usdz,nativeGPU,24,12,0.027714542,7.62939453125e-06,7.62939453125e-06,25.28125,ok,
/Users/markus/GIT/meshdenoiser/build/_deps/tinyusdz-src/models/texture-cat-plane.usdz,reference,54,18,nan,nan,nan,0,failed,"Unable to get neighborhood information, no filtering done...; mesh vertices=54 faces=18 boundary_edges=54 nonmanifold_edges=0 degenerate_faces=0"
/Users/markus/GIT/meshdenoiser/build/_deps/tinyusdz-src/models/texture-cat-plane.usdz,nativeCPU,54,18,nan,nan,nan,0,failed,error: invalidInput; mesh vertices=54 faces=18 boundary_edges=54 nonmanifold_edges=0 degenerate_faces=0
/Users/markus/GIT/meshdenoiser/build/_deps/tinyusdz-src/models/texture-cat-plane.usdz,nativeGPU,54,18,nan,nan,nan,0,failed,error: invalidInput; mesh vertices=54 faces=18 boundary_edges=54 nonmanifold_edges=0 degenerate_faces=0
```

CI exit-code smoke on the same mixed manifest:

```text
--keep-going exited 0
--keep-going --fail-on-error exited 1 after writing all rows
stderr: batch summary: inputs=2 backend_runs=6 failed=3
```

Parity-threshold smoke on `cube.usdz` with intentionally strict `--max-error-threshold 0.000001`:

```text
input,backend,vertices,faces,total_secs,max_error_vs_reference,mean_error_vs_reference,peak_memory_mb,status,error
/Users/markus/GIT/meshdenoiser/build/_deps/tinyusdz-src/models/cube.usdz,nativeCPU,24,12,0.000216708,7.62939453125e-06,7.62939453125e-06,19.671875,failed,max_error 7.62939453125e-06 > 1e-06
stderr: batch summary: inputs=1 backend_runs=1 failed=1
exit: 1
```

Resource-threshold smoke on `cube.usdz` with intentionally strict `--max-memory-mb-threshold 1`:

```text
input,backend,vertices,faces,total_secs,max_error_vs_reference,mean_error_vs_reference,peak_memory_mb,status,error
/Users/markus/GIT/meshdenoiser/build/_deps/tinyusdz-src/models/cube.usdz,nativeCPU,24,12,0.000260125,7.62939453125e-06,7.62939453125e-06,19.78125,failed,peak_memory_mb 19.78125 > 1.0
stderr: batch summary: inputs=1 backend_runs=1 failed=1
exit: 1
```

Local Plinth USDZ sweep over 11 discovered production-style assets:

```text
nativeCPU: inputs=11 ok=9 failed=2
nativeCPU worst max_error=0.00103573 on DOG/Dog.usdz
nativeCPU worst mean_error=5.20859e-05 on DOG/Dog.usdz
nativeCPU max total_secs=1.27343 on Plinth_Comparison/egg/egg_pg.usdz
nativeCPU max peak_memory_mb=522.484 on Plinth_Comparison/egg/egg_pg.usdz

nativeGPU: inputs=9 ok=9 failed=0
nativeGPU worst max_error=0.00103576 on DOG/Dog.usdz
nativeGPU worst mean_error=5.20848e-05 on DOG/Dog.usdz
nativeGPU max total_secs=0.89311 on Plinth_Comparison/egg/egg_pg.usdz
nativeGPU max peak_memory_mb=667.609 on Plinth_Comparison/egg/egg_pg.usdz
```

The two rejected files failed topology validation before denoising:

```text
Cat_12MP.usdz: vertices=55251 faces=99400 boundary_edges=10738 nonmanifold_edges=2 degenerate_faces=0
Hare_48MP.usdz: vertices=124529 faces=239994 boundary_edges=8992 nonmanifold_edges=1 degenerate_faces=0
```

Explicit CI-style gate over the 9 topology-valid assets passed:

```bash
swift run -c release MeshDenoiserBench --input-list path/to/topology-valid-usdz-fixtures.txt --backend nativeGPU --keep-going --fail-on-error --max-error-threshold 0.002 --mean-error-threshold 0.0002 --max-total-seconds-threshold 2 --max-memory-mb-threshold 1024
```

```text
exit: 0
batch summary: inputs=9 backend_runs=9 failed=0
```

Conservative repair sweep over the 2 previously rejected local USDZ assets:

```text
nativeCPU + --repair conservative: inputs=2 ok=2 failed=0
Cat_12MP.usdz: repaired to vertices=49696 faces=99396 max_error=2.2220669052330777e-05 mean_error=4.078098621584903e-06 total_secs=0.911411792 peak_memory_mb=373.78125
Hare_48MP.usdz: repaired to vertices=119997 faces=239992 max_error=3.879942960338667e-05 mean_error=4.653728732670328e-06 total_secs=2.971402042 peak_memory_mb=1174.9375

nativeGPU + --repair conservative: inputs=2 ok=2 failed=0
Cat_12MP.usdz: repaired to vertices=49696 faces=99396 max_error=2.2248139430303127e-05 mean_error=4.0794778216070325e-06 total_secs=0.7236895 peak_memory_mb=481.25
Hare_48MP.usdz: repaired to vertices=119997 faces=239992 max_error=3.878999632433988e-05 mean_error=4.653760781868812e-06 total_secs=2.045737875 peak_memory_mb=1494.921875
```

The repaired native GPU run passed:

```bash
swift run -c release MeshDenoiserBench --input-list path/to/previously-rejected-usdz-fixtures.txt --backend nativeGPU --repair conservative --keep-going --fail-on-error --max-error-threshold 0.002 --mean-error-threshold 0.0002 --max-total-seconds-threshold 5 --max-memory-mb-threshold 2048
```

```text
exit: 0
batch summary: inputs=2 backend_runs=2 failed=0
```

Repair diagnostic columns are appended after `peak_memory_mb` when `--repair conservative` is active:

```text
repair_vertices_before,repair_vertices_after,repair_faces_before,repair_faces_after,repair_boundary_edges_before,repair_boundary_edges_after,repair_nonmanifold_edges_before,repair_nonmanifold_edges_after,repair_degenerate_faces_before,repair_degenerate_faces_after,repair_removed_degenerate_faces,repair_removed_duplicate_faces,repair_removed_nonmanifold_faces,repair_removed_unreferenced_vertices
```

Example repaired `Cat_12MP.usdz` native GPU row:

```text
nativeGPU,49696,99396,0.694456833,2.2248139430303127e-05,4.0794778216070325e-06,480.125,55251,49696,99400,99396,10738,8,2,0,0,0,0,2,2,5555
```

Final repaired release-binary sweep over the 29 local Plinth USDZ files:

```text
rows=29 ok=29 failed=0 bad_field_rows=0 fields=24
max_total_secs=21.701735458 row=Painting.usdz
max_error=0.00026735043502412736 row=Painting.usdz
max_mean_error=2.618257925627628e-05 row=Painting.usdz
max_memory_mb=8540.109375 row=Painting.usdz
budget_5s_2gb_failures=1
```

- [x] Add a benchmark command to README:

```bash
swift run -c release MeshDenoiserBench --faces 81920 --backend all
swift run -c release MeshDenoiserBench --input path/to/model.usdz --backend all
swift run -c release MeshDenoiserBench --input-dir path/to/usdz-fixtures --backend all
swift run -c release MeshDenoiserBench --input-list fixtures.txt --backend all
swift run -c release MeshDenoiserBench --input-dir path/to/usdz-fixtures --backend all --keep-going
swift run -c release MeshDenoiserBench --input-dir path/to/usdz-fixtures --backend all --keep-going --fail-on-error
swift run -c release MeshDenoiserBench --input-dir path/to/usdz-fixtures --backend all --keep-going --fail-on-error --max-error-threshold 0.002 --mean-error-threshold 0.0002 --max-total-seconds-threshold 10 --max-memory-mb-threshold 8192
```

- [x] Evaluate default-backend cutover gates on Apple Silicon:

| Gate | Requirement |
|---|---|
| Public contract | all existing contract tests pass for `.automatic`, `.nativeCPU`, `.reference`; `.nativeGPU` passes or skips |
| Fixture parity | native CPU max vertex error `< 2e-3`, mean `< 2e-4` versus reference |
| GPU parity | native GPU max vertex error `< 2e-3` versus native CPU |
| Performance | native GPU faster than reference at 82k faces |
| Memory | native path peak memory below reference at 328k faces |

Decision: latest benchmark gates pass for `.nativeGPU`: it is faster than `.reference`, parity is within tolerance, 328k generated peak memory is `7242.40625 MB` versus `.reference` at `7658.296875 MB`, and the local 150k-face OBJ file-backed run is also faster and lower-memory than `.reference`. USD/USDZ ModelIO loading is now wired, smoke-tested, batchable through directory or manifest inputs, and failure-tolerant with `--keep-going`. The isolated CG vertex-update benchmark shows the solver is not the dominant native runtime. Conservative repair now also orients shared edges consistently; the two local `Shell_uncompressed.usdz` files that previously failed the OpenMesh reference path with `PolyMeshT::add_face: complex edge` now pass native GPU benchmarking with max error `4.408152381074615e-05` and mean error `8.152645528552612e-06`. Keep the public default at `.reference` until larger committed production-like USDZ fixtures pass parity and memory gates.

- [ ] Change default backend:

```swift
public var backend: Backend = .automatic
```

- [ ] Keep `.reference` public for one release cycle or until the app integration has shipped with native enabled.

- [ ] Run:

```bash
swift test
swift run -c release MeshDenoiserBench --faces 81920 --backend all
```

## Risk Decisions

- Do not port OpenMesh. The native target should operate on flat `[SIMD3<Float>]` and `[SIMD3<UInt32>]` buffers to reduce memory and keep app integration simple.
- Do not use Metal for neighbor search in the first native release. Pair generation is branchy, variable-length, and easier to validate on CPU; the fixed-point filter is the better first GPU target.
- Do not use Metal for the sparse vertex solve in this plan. Apple platforms do not provide a general-purpose sparse direct solver in Metal, and the current CPU CG solve is not the main wall compared with preprocessing/filtering memory.
- Do not port Poisson update or homogeneous/non-normalized iterates. The current Swift package API does not expose them, and the approved native design scopes to the iterative update path.
- Keep float32 through geometry/filtering and double through the sparse solve. This is the practical balance between memory pressure and solver stability.
- Keep `.reference` until the benchmark and tolerance gates pass on the target devices. Removing C++ before the native path is validated would eliminate the only local oracle.

## Final Verification Matrix

Run these before marking the port complete:

```bash
swift test
swift test --filter NativeParityTests
swift test --filter GPUParityTests
swift run -c release MeshDenoiserBench --faces 81920 --backend all
swift run -c release MeshDenoiserBench --faces 327680 --backend nativeGPU
```

Expected final state:

- `MeshDenoiser.denoise` remains source-compatible for existing callers.
- `.automatic` chooses native GPU on Metal devices and native CPU otherwise.
- `.reference` remains available for parity checks and emergency fallback.
- Native CPU and GPU produce finite positions, preserve vertex count/order, report progress per outer iteration, and bridge cancellation to `CancellationError`.
- Native parity thresholds are encoded in tests, not only documented.
- The benchmark documents time, memory, and error against the reference backend.
