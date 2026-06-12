# Native Swift + Metal Port Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Reimplement the SD-filter denoiser in pure Swift + Metal + Accelerate as a `MeshDenoiserNative` backend behind the existing `MeshDenoiserKit` API, with float32 tolerance parity against the C++ reference.

**Architecture:** New dependency-free Swift target `MeshDenoiserNative` implementing the pipeline (connectivity → guidance → neighbor search → static weights → GPU/CPU fixed-point filter → Accelerate Cholesky vertex update). `MeshDenoiserKit` routes via a new `backend` parameter; the C++ backend stays as `.reference` until validated. Spec: `docs/superpowers/specs/2026-06-12-native-swift-metal-port-design.md`. Algorithm source of truth: `src/SDFilter.h`, `src/MeshNormalFilter.h`, `src/MeshNormalDenoising.h`.

**Tech Stack:** Swift 5.9, Metal (MSL 2.x), Accelerate Sparse Solvers, XCTest. No third-party dependencies.

---

## Reference: exact formulas being ported

All verified against the C++ (file:line as of commit fd637b8):

| Quantity | Formula | Source |
|---|---|---|
| η′ (scaled eta) | `eta × mean(‖c_f1−c_f2‖ over interior edges)` | MeshNormalFilter.h:147 |
| search radius | `3η′`, pairs with `d² < r²` (strict) | MeshNormalFilter.h:201,275 |
| static pair weight | `(aᵢ+aⱼ)·exp(−‖gᵢ−gⱼ‖²/(2μ²) − d²/(2η′²))` | SDFilter.h:770 |
| λ rescale | `λ′ = λ·Σa / Σ((aᵢ+aⱼ)·exp(−d²/(2η′²)))` | SDFilter.h:778 |
| weighted init | `s⁰ᵢ·aᵢ·2ν²/λ′` | SDFilter.h:450 |
| dynamic pair weight | `w_p·exp(−‖sᵢ−sⱼ‖²/(2ν²))` | SDFilter.h:477 |
| filter iterate | `sᵢ ← normalize(weightedInitᵢ + Σ_{j∈N(i)} sⱼ·w_dyn(p))` | SDFilter.h:493 |
| filter convergence | `Σaᵢ‖Δsᵢ‖² ≤ Σa·ε²`, ε = 2·sin(0.1·π/180), max 100 iters | SDFilter.h:461, MeshNormalFilter.h:54, SDFilter.h:93 |
| areas | sector area, normalized to mean 1 | MeshNormalFilter.h:450-460 |
| update closeness | `w = closeness·F/V` | MeshNormalFilter.h:466 |
| AᵀA per-face block | `I − J/3` on the face's 3-vertex block (since (I−J/3)ᵀ(I−J/3) = I−J/3) | MeshNormalFilter.h:679-682 |
| update RHS | `w·X0 + AᵀB`, B = per-face projected vertex targets | MeshNormalFilter.h:538 |
| projection (normal ok) | `xⱼ − n·(n·xⱼ)` on mean-centered verts when `n_cur·n_tgt ≥ 0` | MeshNormalFilter.h:509-511 |
| projection (flipped) | project onto principal line in target plane | MeshNormalFilter.h:512-531 |
| update convergence | `rms(Δv) ≤ eps` (eps>0), max `meshUpdateIterations` | MeshNormalFilter.h:549-560 |
| mesh normalize | center at vertex mean, scale bbox diag to 1; restore re-centers by current mean | MeshTypes.h:208-236 |

Scope cuts (CLI never exercises these): `normalize_iterates=false` path, Poisson update, CHOLMOD. Native uses Accelerate Cholesky regardless of `linearSolver` (double precision — normal equations square the condition number; stage is cheap).

## File structure

```
Package.swift                                          (modify: 2 new targets)
Sources/MeshDenoiserNative/
  NativeDenoiser.swift          orchestrator, public entry, validation, normalize/restore
  NativeTypes.swift             NativeDenoiseParameters, NativeDenoiseError, CSRAdjacency
  Parallel.swift                chunked concurrentPerform helper
  MeshConnectivity.swift        normals/areas/centroids, edges, vertex CSR adjacency
  NeighborSearch.swift          uniform-grid pair search
  GuidanceNormals.swift         edge saliency → patch consistency → guidance
  FilterPrecompute.swift        static weights, λ rescale, CSR neighborhood
  FilterCPU.swift               CPU fixed-point loop
  FilterGPU.swift               MetalFilterContext
  FilterKernels.metal           3 compute kernels
  VertexUpdate.swift            SparseCholesky wrapper + projection iterations
Sources/MeshDenoiserKit/
  MeshDenoiseParameters.swift   (modify: backend field)
  MeshDenoiser.swift            (modify: routing)
  MeshDenoiseError.swift        (modify: gpuUnavailable case)
Sources/MeshDenoiserBench/main.swift   benchmark executable (macOS)
Tests/MeshDenoiserKitTests/
  ConnectivityTests.swift, NeighborSearchTests.swift, GuidanceTests.swift,
  FilterPrecomputeTests.swift, FilterCPUTests.swift, VertexUpdateTests.swift,
  NativeParityTests.swift, GPUParityTests.swift
  DenoiseContractTests.swift    (modify: parameterize over backends)
  GoldenParityTests.swift       (modify: pin to .reference explicitly)
```

---

### Task 1: Backend scaffolding

**Files:**
- Modify: `Package.swift`
- Create: `Sources/MeshDenoiserNative/NativeTypes.swift`
- Create: `Sources/MeshDenoiserNative/NativeDenoiser.swift` (stub)
- Modify: `Sources/MeshDenoiserKit/MeshDenoiseParameters.swift`
- Modify: `Sources/MeshDenoiserKit/MeshDenoiseError.swift`
- Modify: `Sources/MeshDenoiserKit/MeshDenoiser.swift`

- [ ] **Step 1: Add targets to Package.swift**

Add to `targets:` (and add `"MeshDenoiserNative"` to `MeshDenoiserKit`'s dependencies):

```swift
        .target(
            name: "MeshDenoiserNative",
            path: "Sources/MeshDenoiserNative"
        ),
```

and change the Kit target to:

```swift
        .target(
            name: "MeshDenoiserKit",
            dependencies: ["CMeshDenoiserCore", "MeshDenoiserNative"],
            path: "Sources/MeshDenoiserKit"
        ),
```

and the test target's dependencies (tests `@testable import` the native module directly):

```swift
        .testTarget(
            name: "MeshDenoiserKitTests",
            dependencies: ["MeshDenoiserKit", "MeshDenoiserNative"],
            path: "Tests/MeshDenoiserKitTests",
            resources: [.copy("Fixtures")]
        ),
```

- [ ] **Step 2: Native types + stub entry point**

`Sources/MeshDenoiserNative/NativeTypes.swift`:

```swift
/// Algorithm parameters for the native backend (mirrors the validated subset of
/// SDFilter::MeshDenoisingParameters; defaults match MeshDenoiserDefaults.txt).
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

    public var isValid: Bool {
        lambda > 0 && eta > 0 && mu > 0 && nu > 0
            && meshUpdateClosenessWeight >= 0 && meshUpdateIterations > 0
            && outerIterations > 0
    }
}

public enum NativeDenoiseError: Error, Equatable {
    case invalidInput
    case invalidParameters
    case solverFailed
    case cancelled
}

/// Compressed sparse rows: values[offsets[i]..<offsets[i+1]] belong to row i.
struct CSRAdjacency {
    var offsets: [UInt32]
    var values: [UInt32]
    subscript(row: Int) -> ArraySlice<UInt32> {
        values[Int(offsets[row])..<Int(offsets[row + 1])]
    }
}
```

`Sources/MeshDenoiserNative/NativeDenoiser.swift` (stub):

```swift
import Metal

public enum NativeDenoiser {
    public static var isGPUAvailable: Bool {
        MTLCreateSystemDefaultDevice() != nil
    }

    /// progress returns false to cancel; called after each completed outer iteration.
    public static func denoise(
        positions: [SIMD3<Float>],
        indices: [UInt32],
        parameters: NativeDenoiseParameters,
        useGPU: Bool,
        progress: ((Int, Int) -> Bool)?
    ) throws -> [SIMD3<Float>] {
        throw NativeDenoiseError.solverFailed // stub — replaced in Task 8
    }
}
```

- [ ] **Step 3: Backend enum + routing in MeshDenoiserKit**

Append to `Sources/MeshDenoiserKit/MeshDenoiseParameters.swift` inside the struct:

```swift
    /// Which implementation runs the filter.
    public enum Backend: Sendable, CaseIterable {
        /// Native GPU if a Metal device exists, else native CPU. (Default once validated.)
        case automatic
        /// Native Metal implementation; throws `.gpuUnavailable` without a Metal device.
        case nativeGPU
        /// Native CPU (parallel Swift + Accelerate) implementation.
        case nativeCPU
        /// The wrapped C++ implementation (test oracle; removed after validation).
        case reference
    }
    public var backend: Backend = .reference   // flipped to .automatic in the final task
```

Add to `MeshDenoiseError` in `Sources/MeshDenoiserKit/MeshDenoiseError.swift`:

```swift
    /// `.nativeGPU` was requested but no Metal device is available.
    case gpuUnavailable
```

In `Sources/MeshDenoiserKit/MeshDenoiser.swift`, add `import MeshDenoiserNative`, rename the existing C-path body of `runSync` to `runReferenceSync` (same signature), and make `runSync` route:

```swift
    private static func runSync(
        positions: [SIMD3<Float>],
        indices: [UInt32],
        parameters: MeshDenoiseParameters,
        session: DenoiseSession
    ) throws -> [SIMD3<Float>] {
        switch parameters.backend {
        case .reference:
            return try runReferenceSync(positions: positions, indices: indices,
                                        parameters: parameters, session: session)
        case .nativeGPU where !NativeDenoiser.isGPUAvailable:
            throw MeshDenoiseError.gpuUnavailable
        case .automatic, .nativeGPU, .nativeCPU:
            let useGPU = parameters.backend != .nativeCPU && NativeDenoiser.isGPUAvailable
            var native = NativeDenoiseParameters()
            native.lambda = parameters.lambda
            native.eta = parameters.eta
            native.mu = parameters.mu
            native.nu = parameters.nu
            native.meshUpdateClosenessWeight = parameters.meshUpdateClosenessWeight
            native.meshUpdateIterations = parameters.meshUpdateIterations
            native.meshUpdateDisplacementEps = parameters.meshUpdateDisplacementEps
            native.outerIterations = parameters.outerIterations
            do {
                return try NativeDenoiser.denoise(
                    positions: positions, indices: indices,
                    parameters: native, useGPU: useGPU
                ) { completed, total in
                    if session.isCancelled { return false }
                    session.progressHandler?(Double(completed) / Double(total))
                    return true
                }
            } catch NativeDenoiseError.cancelled {
                throw CancellationError()
            } catch NativeDenoiseError.invalidInput {
                throw MeshDenoiseError.invalidInput
            } catch NativeDenoiseError.invalidParameters {
                throw MeshDenoiseError.invalidParameters
            } catch let error as NativeDenoiseError {
                _ = error
                throw MeshDenoiseError.solverFailed
            }
        }
    }
```

- [ ] **Step 4: Build and run existing tests (must stay green — default backend unchanged)**

Run: `swift test 2>&1 | grep -E "Executed .* tests"`
Expected: 8 tests, 0 failures.

- [ ] **Step 5: Commit**

```bash
git add Package.swift Sources/MeshDenoiserNative Sources/MeshDenoiserKit
git commit -m "Add MeshDenoiserNative target and backend routing (stub)"
```

---

### Task 2: Mesh connectivity

**Files:**
- Create: `Sources/MeshDenoiserNative/Parallel.swift`
- Create: `Sources/MeshDenoiserNative/MeshConnectivity.swift`
- Create: `Tests/MeshDenoiserKitTests/ConnectivityTests.swift`

- [ ] **Step 1: Write the failing tests**

`Tests/MeshDenoiserKitTests/ConnectivityTests.swift`:

```swift
import XCTest
@testable import MeshDenoiserNative

final class ConnectivityTests: XCTestCase {

    static let octaPositions: [SIMD3<Float>] = [
        [1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1],
    ]
    static let octaIndices: [UInt32] = [
        0, 2, 4,  2, 1, 4,  1, 3, 4,  3, 0, 4,
        2, 0, 5,  1, 2, 5,  3, 1, 5,  0, 3, 5,
    ]

    func testOctahedronCounts() throws {
        let mesh = try MeshConnectivity(positions: Self.octaPositions, indices: Self.octaIndices)
        XCTAssertEqual(mesh.faces.count, 8)
        XCTAssertEqual(mesh.edgeFaces.count, 12)
        XCTAssertTrue(mesh.edgeFaces.allSatisfy { $0.y >= 0 }, "closed mesh: all edges interior")
        for v in 0..<6 {
            XCTAssertEqual(mesh.vertexFaces[v].count, 4)
            XCTAssertEqual(mesh.vertexEdges[v].count, 4)
        }
        // Equilateral octahedron: all areas equal, normalized to mean 1.
        for a in mesh.faceAreas { XCTAssertEqual(a, 1.0, accuracy: 1e-5) }
        // Face 0 = (+x,+y,+z) octant; outward normal = normalize(1,1,1).
        let n = mesh.faceNormals[0]
        let expected = SIMD3<Float>(1, 1, 1) / sqrt(3)
        XCTAssertLessThan(simd_length(n - expected), 1e-5)
        XCTAssertGreaterThan(mesh.averageAdjacentCentroidDistance, 0)
    }

    func testBoundaryEdgeDetected() throws {
        // Two triangles sharing one edge: 5 edges, 1 interior.
        let positions: [SIMD3<Float>] = [[0,0,0], [1,0,0], [0,1,0], [1,1,0]]
        let indices: [UInt32] = [0, 1, 2,  2, 1, 3]
        let mesh = try MeshConnectivity(positions: positions, indices: indices)
        XCTAssertEqual(mesh.edgeFaces.count, 5)
        XCTAssertEqual(mesh.edgeFaces.filter { $0.y >= 0 }.count, 1)
    }

    func testNonManifoldEdgeRejected() {
        let positions: [SIMD3<Float>] = [[0,0,0], [1,0,0], [0,1,0], [0,0,1], [0,-1,0]]
        let indices: [UInt32] = [0, 1, 2,  0, 1, 3,  0, 1, 4] // 3 faces share edge 0-1
        XCTAssertThrowsError(try MeshConnectivity(positions: positions, indices: indices)) {
            XCTAssertEqual($0 as? NativeDenoiseError, .invalidInput)
        }
    }

    func testDegenerateFaceRejected() {
        let positions: [SIMD3<Float>] = [[0,0,0], [1,0,0], [0,1,0]]
        let indices: [UInt32] = [0, 1, 1]
        XCTAssertThrowsError(try MeshConnectivity(positions: positions, indices: indices)) {
            XCTAssertEqual($0 as? NativeDenoiseError, .invalidInput)
        }
    }
}
```

- [ ] **Step 2: Run to verify failure**

Run: `swift test --filter ConnectivityTests 2>&1 | tail -3`
Expected: compile FAIL — `MeshConnectivity` unresolved.

- [ ] **Step 3: Implement**

`Sources/MeshDenoiserNative/Parallel.swift`:

```swift
import Dispatch
import Foundation

/// Chunked concurrentPerform. Bodies receive disjoint index ranges; writes into
/// shared buffers must use the per-range disjointness for safety.
@inline(__always)
func parallelFor(_ count: Int, minChunk: Int = 4096, _ body: (Range<Int>) -> Void) {
    guard count > 0 else { return }
    let maxChunks = max(1, count / minChunk)
    let chunks = min(ProcessInfo.processInfo.activeProcessorCount * 4, maxChunks)
    if chunks <= 1 { body(0..<count); return }
    let chunkSize = (count + chunks - 1) / chunks
    DispatchQueue.concurrentPerform(iterations: chunks) { c in
        let lo = c * chunkSize
        let hi = min(count, lo + chunkSize)
        if lo < hi { body(lo..<hi) }
    }
}
```

`Sources/MeshDenoiserNative/MeshConnectivity.swift`:

```swift
import simd

/// Topology + per-face geometry for a triangle mesh.
/// Mirrors what the C++ port reads from OpenMesh: face normals/areas/centroids,
/// edge→face adjacency, vertex→face and vertex→edge adjacency.
struct MeshConnectivity {
    let vertexCount: Int
    let faces: [SIMD3<UInt32>]
    let faceNormals: [SIMD3<Float>]          // unit (zero for degenerate-area faces)
    let faceAreas: [Float]                   // normalized to mean 1
    let faceCentroids: [SIMD3<Float>]
    let edgeVertices: [SIMD2<UInt32>]        // (min, max) vertex ids
    let edgeFaces: [SIMD2<Int32>]            // adjacent faces; .y == -1 for boundary
    let vertexFaces: CSRAdjacency
    let vertexEdges: CSRAdjacency
    let averageAdjacentCentroidDistance: Double

    init(positions: [SIMD3<Float>], indices: [UInt32]) throws {
        guard !positions.isEmpty, !indices.isEmpty, indices.count % 3 == 0 else {
            throw NativeDenoiseError.invalidInput
        }
        vertexCount = positions.count
        let faceCount = indices.count / 3
        var faces = [SIMD3<UInt32>](repeating: .zero, count: faceCount)
        for f in 0..<faceCount {
            let a = indices[3 * f], b = indices[3 * f + 1], c = indices[3 * f + 2]
            guard a != b, b != c, a != c else { throw NativeDenoiseError.invalidInput }
            faces[f] = SIMD3(a, b, c)
        }
        self.faces = faces

        // Per-face geometry (parallel; areas mean-normalized with Double accumulation).
        var normals = [SIMD3<Float>](repeating: .zero, count: faceCount)
        var rawAreas = [Float](repeating: 0, count: faceCount)
        var centroids = [SIMD3<Float>](repeating: .zero, count: faceCount)
        normals.withUnsafeMutableBufferPointer { nrm in
            rawAreas.withUnsafeMutableBufferPointer { ar in
                centroids.withUnsafeMutableBufferPointer { cen in
                    parallelFor(faceCount) { range in
                        for f in range {
                            let fv = faces[f]
                            let p0 = positions[Int(fv.x)], p1 = positions[Int(fv.y)], p2 = positions[Int(fv.z)]
                            let cr = simd_cross(p1 - p0, p2 - p0)
                            let len = simd_length(cr)
                            nrm[f] = len > 0 ? cr / len : .zero
                            ar[f] = 0.5 * len
                            cen[f] = (p0 + p1 + p2) / 3
                        }
                    }
                }
            }
        }
        var areaSum = 0.0
        for a in rawAreas { areaSum += Double(a) }
        let invMean = Float(Double(faceCount) / max(areaSum, .leastNormalMagnitude))
        self.faceAreas = rawAreas.map { $0 * invMean }
        self.faceNormals = normals
        self.faceCentroids = centroids

        // Edges via dictionary keyed on packed (min,max) vertex pair.
        var edgeIndex = [UInt64: Int32](minimumCapacity: faceCount * 2)
        var edgeVertices: [SIMD2<UInt32>] = []
        var edgeFaces: [SIMD2<Int32>] = []
        edgeVertices.reserveCapacity(faceCount * 3 / 2)
        edgeFaces.reserveCapacity(faceCount * 3 / 2)
        for f in 0..<faceCount {
            let fv = faces[f]
            for (a, b) in [(fv.x, fv.y), (fv.y, fv.z), (fv.z, fv.x)] {
                let lo = min(a, b), hi = max(a, b)
                let key = UInt64(lo) << 32 | UInt64(hi)
                if let e = edgeIndex[key] {
                    guard edgeFaces[Int(e)].y < 0 else { throw NativeDenoiseError.invalidInput } // >2 faces
                    edgeFaces[Int(e)].y = Int32(f)
                } else {
                    edgeIndex[key] = Int32(edgeVertices.count)
                    edgeVertices.append(SIMD2(lo, hi))
                    edgeFaces.append(SIMD2(Int32(f), -1))
                }
            }
        }
        self.edgeVertices = edgeVertices
        self.edgeFaces = edgeFaces

        // Vertex→face and vertex→edge CSR (count, prefix, fill — face/edge order).
        func buildCSR(rowCount: Int, entries: (((Int, UInt32) -> Void) -> Void)) -> CSRAdjacency {
            var degree = [UInt32](repeating: 0, count: rowCount)
            entries { row, _ in degree[row] += 1 }
            var offsets = [UInt32](repeating: 0, count: rowCount + 1)
            for i in 0..<rowCount { offsets[i + 1] = offsets[i] + degree[i] }
            var values = [UInt32](repeating: 0, count: Int(offsets[rowCount]))
            var cursor = offsets
            entries { row, value in
                values[Int(cursor[row])] = value
                cursor[row] += 1
            }
            return CSRAdjacency(offsets: offsets, values: values)
        }
        self.vertexFaces = buildCSR(rowCount: vertexCount) { emit in
            for f in 0..<faceCount {
                let fv = faces[f]
                emit(Int(fv.x), UInt32(f)); emit(Int(fv.y), UInt32(f)); emit(Int(fv.z), UInt32(f))
            }
        }
        self.vertexEdges = buildCSR(rowCount: vertexCount) { emit in
            for e in 0..<edgeVertices.count {
                emit(Int(edgeVertices[e].x), UInt32(e)); emit(Int(edgeVertices[e].y), UInt32(e))
            }
        }

        // Mean centroid distance over interior edges (Double accumulation).
        var dist = 0.0
        var interior = 0
        for ef in edgeFaces where ef.y >= 0 {
            dist += Double(simd_length(centroids[Int(ef.x)] - centroids[Int(ef.y)]))
            interior += 1
        }
        self.averageAdjacentCentroidDistance = interior > 0 ? dist / Double(interior) : 0
    }
}
```

- [ ] **Step 4: Run tests**

Run: `swift test --filter ConnectivityTests 2>&1 | tail -3`
Expected: 4 tests PASS.

- [ ] **Step 5: Commit**

```bash
git add Sources/MeshDenoiserNative Tests/MeshDenoiserKitTests/ConnectivityTests.swift
git commit -m "Add native mesh connectivity with edge adjacency and CSR"
```

---

### Task 3: Neighbor search

**Files:**
- Create: `Sources/MeshDenoiserNative/NeighborSearch.swift`
- Create: `Tests/MeshDenoiserKitTests/NeighborSearchTests.swift`

- [ ] **Step 1: Write the failing test (brute-force oracle on the committed fixture)**

`Tests/MeshDenoiserKitTests/NeighborSearchTests.swift`:

```swift
import XCTest
@testable import MeshDenoiserNative
@testable import MeshDenoiserKit

final class NeighborSearchTests: XCTestCase {

    func testMatchesBruteForceOnFixture() throws {
        let fixtures = Bundle.module.resourceURL!.appendingPathComponent("Fixtures")
        let obj = try OBJLoader.load(fixtures.appendingPathComponent("noisy_icosphere.obj"))
        let positions = obj.positions.map { SIMD3<Float>(Float($0.x), Float($0.y), Float($0.z)) }
        let mesh = try MeshConnectivity(positions: positions, indices: obj.indices)
        let radius = Float(3.0 * 2.2 * mesh.averageAdjacentCentroidDistance)

        let pairs = findNeighborPairs(centroids: mesh.faceCentroids, radius: radius)

        var expected = Set<UInt64>()
        let r2 = radius * radius
        for i in 0..<mesh.faceCentroids.count {
            for j in (i + 1)..<mesh.faceCentroids.count
            where simd_length_squared(mesh.faceCentroids[i] - mesh.faceCentroids[j]) < r2 {
                expected.insert(UInt64(i) << 32 | UInt64(j))
            }
        }
        var got = Set<UInt64>()
        for p in 0..<pairs.count {
            let i = min(pairs.first[p], pairs.second[p]), j = max(pairs.first[p], pairs.second[p])
            got.insert(UInt64(i) << 32 | UInt64(j))
        }
        XCTAssertEqual(got, expected)
        XCTAssertGreaterThan(pairs.count, mesh.faceCentroids.count) // sanity: dense neighborhoods
        for p in 0..<pairs.count {
            let d = simd_length(mesh.faceCentroids[Int(pairs.first[p])] - mesh.faceCentroids[Int(pairs.second[p])])
            XCTAssertEqual(pairs.distance[p], d, accuracy: 1e-6)
        }
    }
}
```

- [ ] **Step 2: Run to verify failure**

Run: `swift test --filter NeighborSearchTests 2>&1 | tail -3`
Expected: compile FAIL — `findNeighborPairs` unresolved.

- [ ] **Step 3: Implement**

`Sources/MeshDenoiserNative/NeighborSearch.swift`:

```swift
import simd

struct NeighborPairs {
    var first: [UInt32] = []
    var second: [UInt32] = []
    var distance: [Float] = []
    var count: Int { first.count }
}

/// Uniform-grid radius search over face centroids. Returns each pair once (i < j
/// in emission order). Deterministic: two passes (count, fill) in face order.
func findNeighborPairs(centroids: [SIMD3<Float>], radius: Float) -> NeighborPairs {
    let n = centroids.count
    guard n > 0, radius > 0 else { return NeighborPairs() }
    var minC = centroids[0]
    for c in centroids { minC = simd_min(minC, c) }
    let inv = 1.0 / radius

    @inline(__always) func cellOf(_ p: SIMD3<Float>) -> SIMD3<Int64> {
        let q = (p - minC) * inv
        return SIMD3<Int64>(Int64(max(0, q.x)), Int64(max(0, q.y)), Int64(max(0, q.z)))
    }
    @inline(__always) func key(_ c: SIMD3<Int64>) -> UInt64 {
        UInt64(c.x & 0x1FFFFF) | (UInt64(c.y & 0x1FFFFF) << 21) | (UInt64(c.z & 0x1FFFFF) << 42)
    }

    var cells = [UInt64: [UInt32]](minimumCapacity: n)
    for i in 0..<n {
        cells[key(cellOf(centroids[i])), default: []].append(UInt32(i))
    }

    let r2 = radius * radius
    // Visit the 27-cell neighborhood of face i, calling body for each candidate j > i in range.
    @inline(__always) func forEachNeighbor(_ i: Int, _ body: (UInt32, Float) -> Void) {
        let ci = centroids[i]
        let cell = cellOf(ci)
        for dx in -1...1 { for dy in -1...1 { for dz in -1...1 {
            guard let bucket = cells[key(cell &+ SIMD3<Int64>(Int64(dx), Int64(dy), Int64(dz)))] else { continue }
            for j in bucket where j > UInt32(i) {
                let d2 = simd_length_squared(ci - centroids[Int(j)])
                if d2 < r2 { body(j, d2.squareRoot()) }
            }
        }}}
    }

    // Pass 1: count per face. Pass 2: fill at prefix offsets. Both parallel, deterministic.
    var counts = [Int](repeating: 0, count: n)
    counts.withUnsafeMutableBufferPointer { cnt in
        parallelFor(n, minChunk: 512) { range in
            for i in range {
                var c = 0
                forEachNeighbor(i) { _, _ in c += 1 }
                cnt[i] = c
            }
        }
    }
    var offsets = [Int](repeating: 0, count: n + 1)
    for i in 0..<n { offsets[i + 1] = offsets[i] + counts[i] }
    let total = offsets[n]

    var result = NeighborPairs(
        first: .init(repeating: 0, count: total),
        second: .init(repeating: 0, count: total),
        distance: .init(repeating: 0, count: total)
    )
    result.first.withUnsafeMutableBufferPointer { f in
        result.second.withUnsafeMutableBufferPointer { s in
            result.distance.withUnsafeMutableBufferPointer { d in
                parallelFor(n, minChunk: 512) { range in
                    for i in range {
                        var w = offsets[i]
                        forEachNeighbor(i) { j, dist in
                            f[w] = UInt32(i); s[w] = j; d[w] = dist; w += 1
                        }
                    }
                }
            }
        }
    }
    return result
}
```

- [ ] **Step 4: Run tests**

Run: `swift test --filter NeighborSearchTests 2>&1 | tail -3`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add Sources/MeshDenoiserNative/NeighborSearch.swift Tests/MeshDenoiserKitTests/NeighborSearchTests.swift
git commit -m "Add grid-based neighbor pair search with brute-force-verified test"
```

---

### Task 4: Guidance normals

**Files:**
- Create: `Sources/MeshDenoiserNative/GuidanceNormals.swift`
- Create: `Tests/MeshDenoiserKitTests/GuidanceTests.swift`

Port of `MeshNormalDenoising::get_initial_data` (MeshNormalDenoising.h:142-285): edge saliency = ‖n_f1−n_f2‖ on interior edges; per face, the candidate patch is the faces/non-boundary edges adjacent to its 3 vertices; patch consistency = (maxSaliency/(1e-9+ΣSaliency))·maxPairwiseNormalDiff; patch average normal = area-weighted, normalized. Each face takes the average normal of the most consistent (minimum score, first wins) patch among its neighborhood faces.

- [ ] **Step 1: Write the failing tests**

`Tests/MeshDenoiserKitTests/GuidanceTests.swift`:

```swift
import XCTest
@testable import MeshDenoiserNative
@testable import MeshDenoiserKit

final class GuidanceTests: XCTestCase {

    /// A flat fan: all face normals identical, so every patch average equals that
    /// normal and guidance must equal it exactly (up to normalization).
    func testFlatMeshGuidanceEqualsNormals() throws {
        // 3x3 vertex grid in z=0, 8 triangles
        var positions: [SIMD3<Float>] = []
        for y in 0..<3 { for x in 0..<3 { positions.append([Float(x), Float(y), 0]) } }
        var indices: [UInt32] = []
        for y in 0..<2 { for x in 0..<2 {
            let i = UInt32(y * 3 + x)
            indices += [i, i + 1, i + 3,  i + 1, i + 4, i + 3]
        }}
        let mesh = try MeshConnectivity(positions: positions, indices: indices)
        let guidance = computeGuidanceNormals(mesh)
        XCTAssertEqual(guidance.count, mesh.faces.count)
        for g in guidance {
            XCTAssertLessThan(simd_length(g - SIMD3<Float>(0, 0, 1)), 1e-6)
        }
    }

    func testGuidanceIsUnitLengthOnNoisyFixture() throws {
        let fixtures = Bundle.module.resourceURL!.appendingPathComponent("Fixtures")
        let obj = try OBJLoader.load(fixtures.appendingPathComponent("noisy_icosphere.obj"))
        let positions = obj.positions.map { SIMD3<Float>(Float($0.x), Float($0.y), Float($0.z)) }
        let mesh = try MeshConnectivity(positions: positions, indices: obj.indices)
        let guidance = computeGuidanceNormals(mesh)
        for g in guidance {
            XCTAssertEqual(simd_length(g), 1.0, accuracy: 1e-4)
        }
    }
}
```

- [ ] **Step 2: Run to verify failure**

Run: `swift test --filter GuidanceTests 2>&1 | tail -3`
Expected: compile FAIL.

- [ ] **Step 3: Implement**

`Sources/MeshDenoiserNative/GuidanceNormals.swift`:

```swift
import simd

/// Patch-based guidance normals (MeshNormalDenoising::get_initial_data port).
func computeGuidanceNormals(_ mesh: MeshConnectivity) -> [SIMD3<Float>] {
    let faceCount = mesh.faces.count
    let edgeCount = mesh.edgeFaces.count

    var edgeSaliency = [Float](repeating: 0, count: edgeCount)
    edgeSaliency.withUnsafeMutableBufferPointer { sal in
        parallelFor(edgeCount) { range in
            for e in range {
                let ef = mesh.edgeFaces[e]
                if ef.y >= 0 {
                    sal[e] = simd_length(mesh.faceNormals[Int(ef.x)] - mesh.faceNormals[Int(ef.y)])
                }
            }
        }
    }

    // Faces and non-boundary edges adjacent to the 3 vertices of face i, deduped
    // (patches are small, ≤~30 entries — linear dedup beats stamp arrays here).
    func collectPatch(of i: Int, faces: inout [UInt32], edges: inout [UInt32]) {
        faces.removeAll(keepingCapacity: true)
        edges.removeAll(keepingCapacity: true)
        let fv = mesh.faces[i]
        for v in [Int(fv.x), Int(fv.y), Int(fv.z)] {
            for f in mesh.vertexFaces[v] where !faces.contains(f) { faces.append(f) }
            for e in mesh.vertexEdges[v]
            where mesh.edgeFaces[Int(e)].y >= 0 && !edges.contains(e) { edges.append(e) }
        }
    }

    var patchConsistency = [Float](repeating: 0, count: faceCount)
    var patchAvgNormal = [SIMD3<Float>](repeating: .zero, count: faceCount)
    patchConsistency.withUnsafeMutableBufferPointer { cons in
        patchAvgNormal.withUnsafeMutableBufferPointer { avg in
            parallelFor(faceCount, minChunk: 1024) { range in
                var pf: [UInt32] = []; pf.reserveCapacity(32)
                var pe: [UInt32] = []; pe.reserveCapacity(32)
                for i in range {
                    collectPatch(of: i, faces: &pf, edges: &pe)

                    var maxSal: Float = 0, sumSal: Float = 0
                    for e in pe { maxSal = max(maxSal, edgeSaliency[Int(e)]); sumSal += edgeSaliency[Int(e)] }
                    var consistency: Float = pe.isEmpty ? 0 : maxSal / (1e-9 + sumSal)

                    var maxDiff: Float = 0
                    for a in 0..<pf.count {
                        for b in (a + 1)..<pf.count {
                            maxDiff = max(maxDiff, simd_length(
                                mesh.faceNormals[Int(pf[a])] - mesh.faceNormals[Int(pf[b])]))
                        }
                    }
                    consistency *= maxDiff
                    cons[i] = consistency

                    var acc = SIMD3<Float>.zero
                    for f in pf { acc += mesh.faceNormals[Int(f)] * mesh.faceAreas[Int(f)] }
                    let len = simd_length(acc)
                    avg[i] = len > 0 ? acc / len : .zero
                }
            }
        }
    }

    var guidance = [SIMD3<Float>](repeating: .zero, count: faceCount)
    guidance.withUnsafeMutableBufferPointer { gd in
        parallelFor(faceCount, minChunk: 1024) { range in
            var pf: [UInt32] = []; pf.reserveCapacity(32)
            var pe: [UInt32] = []; pe.reserveCapacity(32)
            for i in range {
                collectPatch(of: i, faces: &pf, edges: &pe)
                var best = pf[0]
                var bestScore = patchConsistency[Int(pf[0])]
                for f in pf.dropFirst() where patchConsistency[Int(f)] < bestScore {
                    best = f; bestScore = patchConsistency[Int(f)]
                }
                gd[i] = patchAvgNormal[Int(best)]
            }
        }
    }
    return guidance
}
```

- [ ] **Step 4: Run tests**

Run: `swift test --filter GuidanceTests 2>&1 | tail -3`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add Sources/MeshDenoiserNative/GuidanceNormals.swift Tests/MeshDenoiserKitTests/GuidanceTests.swift
git commit -m "Add patch-based guidance normal computation"
```

---

### Task 5: Filter precompute (static weights, λ rescale, CSR)

**Files:**
- Create: `Sources/MeshDenoiserNative/FilterPrecompute.swift`
- Create: `Tests/MeshDenoiserKitTests/FilterPrecomputeTests.swift`

- [ ] **Step 1: Write the failing test (hand-computed two-triangle case)**

`Tests/MeshDenoiserKitTests/FilterPrecomputeTests.swift`:

```swift
import XCTest
@testable import MeshDenoiserNative

final class FilterPrecomputeTests: XCTestCase {

    func testTwoTriangleWeightsMatchFormula() throws {
        // Two coplanar triangles sharing an edge — one neighbor pair.
        let positions: [SIMD3<Float>] = [[0,0,0], [1,0,0], [0,1,0], [1,1,0]]
        let indices: [UInt32] = [0, 1, 2,  2, 1, 3]
        let mesh = try MeshConnectivity(positions: positions, indices: indices)
        let guidance = computeGuidanceNormals(mesh)
        var params = NativeDenoiseParameters()
        let etaScaled = params.eta * mesh.averageAdjacentCentroidDistance
        let pairs = findNeighborPairs(centroids: mesh.faceCentroids, radius: Float(3 * etaScaled))
        XCTAssertEqual(pairs.count, 1)

        let ops = try precomputeFilterOperands(
            mesh: mesh, guidance: guidance, pairs: pairs, parameters: params)

        // Hand-compute: a0 == a1 == 1 (mean-normalized equal areas), g0 == g1 (flat).
        let d = Double(pairs.distance[0])
        let hS = -0.5 / (etaScaled * etaScaled)
        let expectedStatic = 2.0 * exp(hS * d * d)   // guidance term = exp(0) = 1
        XCTAssertEqual(Double(ops.staticWeights[0]), expectedStatic, accuracy: 1e-6)

        // λ′ = λ·Σa/Σ((aᵢ+aⱼ)exp(hS d²)) here = 0.15·2/expectedStatic
        // weightedInit_i = n_i·aᵢ·2ν²/λ′
        let lambdaR = 0.15 * 2.0 / expectedStatic
        let scale = 2.0 * 0.25 * 0.25 / lambdaR
        XCTAssertEqual(Double(ops.weightedInit[0].z), scale, accuracy: 1e-5)

        // CSR: each face sees the other once, both referencing pair 0.
        XCTAssertEqual(ops.csrOffsets, [0, 1, 2])
        XCTAssertEqual(ops.csrEntries[0], SIMD2<UInt32>(1, 0))
        XCTAssertEqual(ops.csrEntries[1], SIMD2<UInt32>(0, 0))
        XCTAssertEqual(ops.pairFaces.count, 1)
        _ = params
    }
}
```

- [ ] **Step 2: Run to verify failure**

Run: `swift test --filter FilterPrecomputeTests 2>&1 | tail -3`
Expected: compile FAIL.

- [ ] **Step 3: Implement**

`Sources/MeshDenoiserNative/FilterPrecompute.swift`:

```swift
import Darwin
import simd

struct FilterOperands {
    var pairFaces: [SIMD2<UInt32>]      // (i, j) per pair
    var staticWeights: [Float]          // per pair
    var csrOffsets: [UInt32]            // faceCount + 1
    var csrEntries: [SIMD2<UInt32>]     // (neighbor face, pair index); 2 × pairCount
    var weightedInit: [SIMD3<Float>]    // s⁰ᵢ · aᵢ · 2ν²/λ′
    var hNu: Float                      // −1/(2ν²)
    var maxIterations: Int              // 100 (SDFilter Parameters default)
    var convergenceThreshold: Double    // Σa · (2·sin(0.1°·π/180))²
}

func precomputeFilterOperands(
    mesh: MeshConnectivity,
    guidance: [SIMD3<Float>],
    pairs: NeighborPairs,
    parameters: NativeDenoiseParameters
) throws -> FilterOperands {
    let faceCount = mesh.faces.count
    let pairCount = pairs.count
    guard pairCount > 0 else { throw NativeDenoiseError.solverFailed } // matches C++ initialize_filter failure

    let etaScaled = parameters.eta * mesh.averageAdjacentCentroidDistance
    let hSpatial = -0.5 / (etaScaled * etaScaled)
    let hGuidance = -0.5 / (parameters.mu * parameters.mu)

    var staticWeights = [Float](repeating: 0, count: pairCount)
    var areaSpatialPartial = [Double](repeating: 0, count: pairCount)
    staticWeights.withUnsafeMutableBufferPointer { sw in
        areaSpatialPartial.withUnsafeMutableBufferPointer { asp in
            parallelFor(pairCount) { range in
                for p in range {
                    let i = Int(pairs.first[p]), j = Int(pairs.second[p])
                    let aSum = Double(mesh.faceAreas[i]) + Double(mesh.faceAreas[j])
                    let d = Double(pairs.distance[p])
                    let g = guidance[i] - guidance[j]
                    asp[p] = aSum * Foundation_exp(hSpatial * d * d)
                    sw[p] = Float(aSum * Foundation_exp(
                        hGuidance * Double(simd_length_squared(g)) + hSpatial * d * d))
                }
            }
        }
    }
    var areaSpatialSum = 0.0
    for v in areaSpatialPartial { areaSpatialSum += v }
    var areaSum = 0.0
    for a in mesh.faceAreas { areaSum += Double(a) }
    let lambdaRescaled = parameters.lambda * areaSum / areaSpatialSum

    let initScale = 2.0 * parameters.nu * parameters.nu / lambdaRescaled
    var weightedInit = [SIMD3<Float>](repeating: .zero, count: faceCount)
    for i in 0..<faceCount {
        weightedInit[i] = mesh.faceNormals[i] * Float(Double(mesh.faceAreas[i]) * initScale)
    }

    // CSR over both pair directions (SDFilter.h:780-812).
    var degree = [UInt32](repeating: 0, count: faceCount)
    for p in 0..<pairCount {
        degree[Int(pairs.first[p])] += 1
        degree[Int(pairs.second[p])] += 1
    }
    var csrOffsets = [UInt32](repeating: 0, count: faceCount + 1)
    for i in 0..<faceCount { csrOffsets[i + 1] = csrOffsets[i] + degree[i] }
    var csrEntries = [SIMD2<UInt32>](repeating: .zero, count: 2 * pairCount)
    var cursor = csrOffsets
    var pairFaces = [SIMD2<UInt32>](repeating: .zero, count: pairCount)
    for p in 0..<pairCount {
        let i = pairs.first[p], j = pairs.second[p]
        pairFaces[p] = SIMD2(i, j)
        csrEntries[Int(cursor[Int(i)])] = SIMD2(j, UInt32(p)); cursor[Int(i)] += 1
        csrEntries[Int(cursor[Int(j)])] = SIMD2(i, UInt32(p)); cursor[Int(j)] += 1
    }

    let eps = 2.0 * sin(0.1 * Double.pi / 180.0)   // MeshFilterParameters ctor
    return FilterOperands(
        pairFaces: pairFaces,
        staticWeights: staticWeights,
        csrOffsets: csrOffsets,
        csrEntries: csrEntries,
        weightedInit: weightedInit,
        hNu: Float(-0.5 / (parameters.nu * parameters.nu)),
        maxIterations: 100,
        convergenceThreshold: areaSum * eps * eps
    )
}

@inline(__always) private func Foundation_exp(_ x: Double) -> Double { Darwin.exp(x) }
```

- [ ] **Step 4: Run tests**

Run: `swift test --filter FilterPrecomputeTests 2>&1 | tail -3`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add Sources/MeshDenoiserNative/FilterPrecompute.swift Tests/MeshDenoiserKitTests/FilterPrecomputeTests.swift
git commit -m "Add filter precompute: static weights, lambda rescale, CSR"
```

---

### Task 6: CPU fixed-point filter

**Files:**
- Create: `Sources/MeshDenoiserNative/FilterCPU.swift`
- Create: `Tests/MeshDenoiserKitTests/FilterCPUTests.swift`

- [ ] **Step 1: Write the failing test**

`Tests/MeshDenoiserKitTests/FilterCPUTests.swift`:

```swift
import XCTest
@testable import MeshDenoiserNative
@testable import MeshDenoiserKit

final class FilterCPUTests: XCTestCase {

    func testFlatMeshNormalsAreFixedPoint() throws {
        // All normals identical → the iterate is already the fixed point; must
        // converge on iteration 1 with unchanged normals.
        var positions: [SIMD3<Float>] = []
        for y in 0..<4 { for x in 0..<4 { positions.append([Float(x), Float(y), 0]) } }
        var indices: [UInt32] = []
        for y in 0..<3 { for x in 0..<3 {
            let i = UInt32(y * 4 + x)
            indices += [i, i + 1, i + 4,  i + 1, i + 5, i + 4]
        }}
        let mesh = try MeshConnectivity(positions: positions, indices: indices)
        let guidance = computeGuidanceNormals(mesh)
        let params = NativeDenoiseParameters()
        let radius = Float(3 * params.eta * mesh.averageAdjacentCentroidDistance)
        let pairs = findNeighborPairs(centroids: mesh.faceCentroids, radius: radius)
        let ops = try precomputeFilterOperands(mesh: mesh, guidance: guidance, pairs: pairs, parameters: params)

        let out = try runFilterCPU(initialNormals: mesh.faceNormals, areas: mesh.faceAreas,
                                   operands: ops, isCancelled: { false })
        for n in out {
            XCTAssertLessThan(simd_length(n - SIMD3<Float>(0, 0, 1)), 1e-5)
        }
    }

    func testCancellationThrows() throws {
        let positions: [SIMD3<Float>] = [[0,0,0], [1,0,0], [0,1,0], [1,1,0]]
        let indices: [UInt32] = [0, 1, 2,  2, 1, 3]
        let mesh = try MeshConnectivity(positions: positions, indices: indices)
        let guidance = computeGuidanceNormals(mesh)
        let params = NativeDenoiseParameters()
        let radius = Float(3 * params.eta * mesh.averageAdjacentCentroidDistance)
        let pairs = findNeighborPairs(centroids: mesh.faceCentroids, radius: radius)
        let ops = try precomputeFilterOperands(mesh: mesh, guidance: guidance, pairs: pairs, parameters: params)
        XCTAssertThrowsError(try runFilterCPU(initialNormals: mesh.faceNormals, areas: mesh.faceAreas,
                                              operands: ops, isCancelled: { true })) {
            XCTAssertEqual($0 as? NativeDenoiseError, .cancelled)
        }
    }
}
```

- [ ] **Step 2: Run to verify failure**

Run: `swift test --filter FilterCPUTests 2>&1 | tail -3`
Expected: compile FAIL.

- [ ] **Step 3: Implement**

`Sources/MeshDenoiserNative/FilterCPU.swift`:

```swift
import Darwin
import simd

/// CPU fixed-point solver (SDFilter::fixedpoint_solver port, float32, parallel).
func runFilterCPU(
    initialNormals: [SIMD3<Float>],
    areas: [Float],
    operands: FilterOperands,
    isCancelled: () -> Bool
) throws -> [SIMD3<Float>] {
    let faceCount = initialNormals.count
    let pairCount = operands.pairFaces.count
    var signals = initialNormals
    var next = [SIMD3<Float>](repeating: .zero, count: faceCount)
    var pairWeights = [Float](repeating: 0, count: pairCount)
    var dispSq = [Float](repeating: 0, count: faceCount)

    for _ in 1...operands.maxIterations {
        if isCancelled() { throw NativeDenoiseError.cancelled }

        signals.withUnsafeBufferPointer { s in
            pairWeights.withUnsafeMutableBufferPointer { pw in
                parallelFor(pairCount) { range in
                    for p in range {
                        let pr = operands.pairFaces[p]
                        let d = s[Int(pr.x)] - s[Int(pr.y)]
                        pw[p] = operands.staticWeights[p] * expf(operands.hNu * simd_length_squared(d))
                    }
                }
            }
            next.withUnsafeMutableBufferPointer { out in
                dispSq.withUnsafeMutableBufferPointer { dsp in
                    pairWeights.withUnsafeBufferPointer { pw in
                        parallelFor(faceCount, minChunk: 1024) { range in
                            for i in range {
                                var acc = operands.weightedInit[i]
                                for k in Int(operands.csrOffsets[i])..<Int(operands.csrOffsets[i + 1]) {
                                    let e = operands.csrEntries[k]
                                    acc += s[Int(e.x)] * pw[Int(e.y)]
                                }
                                let len = simd_length(acc)
                                let n = len > 0 ? acc / len : SIMD3<Float>.zero
                                out[i] = n
                                dsp[i] = areas[i] * simd_length_squared(n - s[i])
                            }
                        }
                    }
                }
            }
        }
        swap(&signals, &next)

        var disp = 0.0
        for v in dispSq { disp += Double(v) }
        if disp <= operands.convergenceThreshold { break }
    }
    return signals
}
```

- [ ] **Step 4: Run tests**

Run: `swift test --filter FilterCPUTests 2>&1 | tail -3`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add Sources/MeshDenoiserNative/FilterCPU.swift Tests/MeshDenoiserKitTests/FilterCPUTests.swift
git commit -m "Add parallel CPU fixed-point filter"
```

---

### Task 7: Vertex update (Accelerate sparse Cholesky)

**Files:**
- Create: `Sources/MeshDenoiserNative/VertexUpdate.swift`
- Create: `Tests/MeshDenoiserKitTests/VertexUpdateTests.swift`

- [ ] **Step 1: Write the failing tests**

`Tests/MeshDenoiserKitTests/VertexUpdateTests.swift`:

```swift
import XCTest
@testable import MeshDenoiserNative

final class VertexUpdateTests: XCTestCase {

    static let octaPositions: [SIMD3<Float>] = [
        [1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1],
    ]
    static let octaIndices: [UInt32] = [
        0, 2, 4,  2, 1, 4,  1, 3, 4,  3, 0, 4,
        2, 0, 5,  1, 2, 5,  3, 1, 5,  0, 3, 5,
    ]

    /// Targets equal to current normals → every face's mean-centered vertices
    /// already lie in the target plane → positions are a fixed point.
    func testCurrentNormalsAreFixedPoint() throws {
        let mesh = try MeshConnectivity(positions: Self.octaPositions, indices: Self.octaIndices)
        let w = 0.001 * Double(mesh.faces.count) / Double(mesh.vertexCount)
        let updater = try VertexUpdater(faces: mesh.faces, vertexCount: mesh.vertexCount, closenessWeight: w)
        let out = try updater.update(positions: Self.octaPositions, targetNormals: mesh.faceNormals,
                                     iterations: 20, displacementEps: 0, isCancelled: { false })
        for (a, b) in zip(out, Self.octaPositions) {
            XCTAssertLessThan(simd_length(a - b), 1e-5)
        }
    }

    /// Slightly perturbed positions with the clean octahedron normals as targets
    /// must move vertices back toward the clean shape.
    func testUpdateReducesDeviation() throws {
        let noise: [SIMD3<Float>] = [
            [0.02, -0.01, 0.01], [-0.01, 0.02, -0.02], [0.01, 0.01, -0.01],
            [-0.02, -0.01, 0.02], [0.01, -0.02, 0.01], [-0.01, 0.01, -0.02],
        ]
        let noisy = zip(Self.octaPositions, noise).map { $0 + $1 }
        let cleanMesh = try MeshConnectivity(positions: Self.octaPositions, indices: Self.octaIndices)
        let w = 0.001 * Double(cleanMesh.faces.count) / Double(cleanMesh.vertexCount)
        let updater = try VertexUpdater(faces: cleanMesh.faces, vertexCount: 6, closenessWeight: w)
        let out = try updater.update(positions: noisy, targetNormals: cleanMesh.faceNormals,
                                     iterations: 20, displacementEps: 0, isCancelled: { false })

        func deviation(_ pts: [SIMD3<Float>]) -> Float {
            // Distance of each face's vertices from its target plane (through the centroid).
            var total: Float = 0
            for (f, fv) in cleanMesh.faces.enumerated() {
                let pts3 = [pts[Int(fv.x)], pts[Int(fv.y)], pts[Int(fv.z)]]
                let centroid = (pts3[0] + pts3[1] + pts3[2]) / 3
                for p in pts3 { total += abs(simd_dot(p - centroid, cleanMesh.faceNormals[f])) }
            }
            return total
        }
        XCTAssertLessThan(deviation(out), deviation(noisy) * 0.5)
    }
}
```

- [ ] **Step 2: Run to verify failure**

Run: `swift test --filter VertexUpdateTests 2>&1 | tail -3`
Expected: compile FAIL.

- [ ] **Step 3: Implement**

`Sources/MeshDenoiserNative/VertexUpdate.swift`:

```swift
import Accelerate
import simd

/// Iterative mesh vertex update (MeshNormalFilter::iterative_mesh_update port).
/// Factorizes AᵀA + wI once (double precision); reusable across outer iterations.
final class VertexUpdater {
    private let faces: [SIMD3<UInt32>]
    private let vertexCount: Int
    private let closenessWeight: Double
    private var factorization: SparseOpaqueFactorization_Double

    init(faces: [SIMD3<UInt32>], vertexCount: Int, closenessWeight: Double) throws {
        self.faces = faces
        self.vertexCount = vertexCount
        self.closenessWeight = closenessWeight

        // COO lower triangle of M = AᵀA + wI. Per-face AᵀA block = I − J/3:
        // diag 2/3, off-diag −1/3 per unordered vertex pair. Duplicates are
        // summed by SparseConvertFromCoordinate.
        var rows: [Int32] = [], cols: [Int32] = [], vals: [Double] = []
        rows.reserveCapacity(faces.count * 6 + vertexCount)
        cols.reserveCapacity(faces.count * 6 + vertexCount)
        vals.reserveCapacity(faces.count * 6 + vertexCount)
        for fv in faces {
            let v = [Int32(fv.x), Int32(fv.y), Int32(fv.z)]
            for j in 0..<3 {
                rows.append(v[j]); cols.append(v[j]); vals.append(2.0 / 3.0)
                for k in (j + 1)..<3 {
                    rows.append(max(v[j], v[k])); cols.append(min(v[j], v[k])); vals.append(-1.0 / 3.0)
                }
            }
        }
        for i in 0..<vertexCount {
            rows.append(Int32(i)); cols.append(Int32(i)); vals.append(closenessWeight)
        }

        var attributes = SparseAttributes_t()
        attributes.triangle = SparseLowerTriangle
        attributes.kind = SparseSymmetric
        let matrix = rows.withUnsafeBufferPointer { r in
            cols.withUnsafeBufferPointer { c in
                vals.withUnsafeBufferPointer { v in
                    SparseConvertFromCoordinate(Int32(vertexCount), Int32(vertexCount),
                                                vals.count, 1, attributes,
                                                r.baseAddress!, c.baseAddress!, v.baseAddress!)
                }
            }
        }
        defer { SparseCleanup(matrix) }
        factorization = SparseFactor(SparseFactorizationCholesky, matrix)
        guard factorization.status == SparseStatusOK else {
            SparseCleanup(factorization)
            throw NativeDenoiseError.solverFailed
        }
    }

    deinit { SparseCleanup(factorization) }

    func update(
        positions: [SIMD3<Float>],
        targetNormals: [SIMD3<Float>],
        iterations: Int,
        displacementEps: Double,
        isCancelled: () -> Bool
    ) throws -> [SIMD3<Float>] {
        let V = vertexCount
        var pos = positions.map { SIMD3<Double>($0) }
        let x0 = pos
        var rhs = [Double](repeating: 0, count: V * 3)   // column-major V×3

        for _ in 0..<iterations {
            if isCancelled() { throw NativeDenoiseError.cancelled }

            // Per-face projected targets, scattered directly into AᵀB (sequential;
            // double-precision, ~ms even at 1M faces) — plus the closeness term.
            var atb = [SIMD3<Double>](repeating: .zero, count: V)
            for (f, fv) in faces.enumerated() {
                let idx = [Int(fv.x), Int(fv.y), Int(fv.z)]
                let p0 = pos[idx[0]], p1 = pos[idx[1]], p2 = pos[idx[2]]
                let centroid = (p0 + p1 + p2) / 3
                let x = [p0 - centroid, p1 - centroid, p2 - centroid]
                let crossN = simd_cross(p1 - p0, p2 - p0)
                let crossLen = simd_length(crossN)
                let current = crossLen > 0 ? crossN / crossLen : SIMD3<Double>.zero
                let target = SIMD3<Double>(targetNormals[f])

                var b: [SIMD3<Double>]
                if simd_dot(current, target) >= 0 {
                    b = x.map { $0 - target * simd_dot(target, $0) }
                } else {
                    // Project onto the principal line within the target plane.
                    let t: SIMD3<Double> = abs(target.x) < 0.9 ? [1, 0, 0] : [0, 1, 0]
                    let u1 = simd_normalize(simd_cross(target, t))
                    let u2 = simd_cross(target, u1)
                    let l = x.map { SIMD2<Double>(simd_dot(u1, $0), simd_dot(u2, $0)) }
                    var a = 0.0, bb = 0.0, c = 0.0
                    for li in l { a += li.x * li.x; bb += li.x * li.y; c += li.y * li.y }
                    let lam = 0.5 * (a + c) + (0.25 * (a - c) * (a - c) + bb * bb).squareRoot()
                    var v2 = SIMD2<Double>(lam - c, bb)
                    let v2len = simd_length(v2)
                    v2 = v2len > 1e-30 ? v2 / v2len : SIMD2<Double>(1, 0)
                    let dir = u1 * v2.x + u2 * v2.y
                    b = x.map { dir * simd_dot(dir, $0) }
                }
                // AᵀB row for vertex k of this face: b_k − mean(b). (Column k of
                // the mean-centering matrix is e_k − 1/3.)
                let mean = (b[0] + b[1] + b[2]) / 3
                for k in 0..<3 { atb[idx[k]] += b[k] - mean }
            }

            rhs.withUnsafeMutableBufferPointer { r in
                for v in 0..<V {
                    let value = atb[v] + x0[v] * closenessWeight
                    r[v] = value.x; r[V + v] = value.y; r[2 * V + v] = value.z
                }
                let X = DenseMatrix_Double(rowCount: Int32(V), columnCount: 3,
                                           columnStride: Int32(V),
                                           attributes: SparseAttributes_t(),
                                           data: r.baseAddress!)
                SparseSolve(factorization, X)
            }

            var dispSum = 0.0
            for v in 0..<V {
                let newPos = SIMD3<Double>(rhs[v], rhs[V + v], rhs[2 * V + v])
                dispSum += simd_length_squared(newPos - pos[v])
                pos[v] = newPos
            }
            if displacementEps > 0, (dispSum / Double(V)).squareRoot() <= displacementEps {
                break
            }
        }
        return pos.map { SIMD3<Float>($0) }
    }
}
```

- [ ] **Step 4: Run tests**

Run: `swift test --filter VertexUpdateTests 2>&1 | tail -3`
Expected: PASS. (If `SparseConvertFromCoordinate`'s argument order trips the compiler, check the header: `SparseConvertFromCoordinate(rowCount:columnCount:blockCount:blockSize:attributes:row:column:data:)` — adjust call to match the SDK's exact signature.)

- [ ] **Step 5: Commit**

```bash
git add Sources/MeshDenoiserNative/VertexUpdate.swift Tests/MeshDenoiserKitTests/VertexUpdateTests.swift
git commit -m "Add Accelerate sparse Cholesky vertex update"
```

---

### Task 8: Orchestrator + end-to-end CPU parity

**Files:**
- Modify: `Sources/MeshDenoiserNative/NativeDenoiser.swift` (replace stub)
- Create: `Tests/MeshDenoiserKitTests/NativeParityTests.swift`
- Modify: `Tests/MeshDenoiserKitTests/DenoiseContractTests.swift`
- Modify: `Tests/MeshDenoiserKitTests/GoldenParityTests.swift`

- [ ] **Step 1: Write the failing parity tests**

`Tests/MeshDenoiserKitTests/NativeParityTests.swift`:

```swift
import simd
import XCTest
@testable import MeshDenoiserKit

final class NativeParityTests: XCTestCase {

    func loadFixture() throws -> (positions: [SIMD3<Float>], golden: [SIMD3<Double>], indices: [UInt32]) {
        let fixtures = Bundle.module.resourceURL!.appendingPathComponent("Fixtures")
        let input = try OBJLoader.load(fixtures.appendingPathComponent("noisy_icosphere.obj"))
        let golden = try OBJLoader.load(fixtures.appendingPathComponent("golden_denoised.obj"))
        return (input.positions.map { SIMD3<Float>(Float($0.x), Float($0.y), Float($0.z)) },
                golden.positions, input.indices)
    }

    func errors(_ got: [SIMD3<Float>], _ want: [SIMD3<Double>]) -> (max: Double, mean: Double) {
        var maxE = 0.0, sum = 0.0
        for (g, w) in zip(got, want) {
            let e = simd_length(SIMD3<Double>(g) - w)
            maxE = max(maxE, e); sum += e
        }
        return (maxE, sum / Double(got.count))
    }

    /// Native CPU vs the CLI double-precision golden output. Unit-scale mesh;
    /// gates per the design spec (float32 tolerance parity).
    func testNativeCPUMatchesGoldenWithinTolerance() async throws {
        let (positions, golden, indices) = try loadFixture()
        var params = MeshDenoiseParameters()
        params.backend = .nativeCPU
        let result = try await MeshDenoiser.denoise(positions: positions, indices: indices, parameters: params)
        let e = errors(result, golden)
        print("native-vs-golden: max=\(e.max) mean=\(e.mean)")   // recorded in commit message
        XCTAssertLessThan(e.max, 2e-3)
        XCTAssertLessThan(e.mean, 2e-4)
    }

    /// Native CPU vs the in-process C++ reference backend on identical input.
    func testNativeCPUMatchesReferenceBackend() async throws {
        let (positions, _, indices) = try loadFixture()
        var ref = MeshDenoiseParameters(); ref.backend = .reference
        var nat = MeshDenoiseParameters(); nat.backend = .nativeCPU
        let a = try await MeshDenoiser.denoise(positions: positions, indices: indices, parameters: ref)
        let b = try await MeshDenoiser.denoise(positions: positions, indices: indices, parameters: nat)
        let e = errors(b, a.map { SIMD3<Double>($0) })
        XCTAssertLessThan(e.max, 2e-3)
    }

    /// Multiple outer iterations exercise re-preprocessing and factorization reuse.
    func testNativeCPUOuterIterationsMatchReference() async throws {
        let (positions, _, indices) = try loadFixture()
        var ref = MeshDenoiseParameters(); ref.backend = .reference; ref.outerIterations = 3
        var nat = MeshDenoiseParameters(); nat.backend = .nativeCPU; nat.outerIterations = 3
        let a = try await MeshDenoiser.denoise(positions: positions, indices: indices, parameters: ref)
        let b = try await MeshDenoiser.denoise(positions: positions, indices: indices, parameters: nat)
        let e = errors(b, a.map { SIMD3<Double>($0) })
        XCTAssertLessThan(e.max, 5e-3)   // tolerances compound over outer iterations
    }
}
```

Parameterize the contract tests: in `DenoiseContractTests.swift`, add inside the class and convert each existing test body to run per backend (shown for two tests; apply the same wrapper to the NaN / out-of-range / empty / negative-lambda / cancellation / progress tests):

```swift
    static var backendsUnderTest: [MeshDenoiseParameters.Backend] {
        var backends: [MeshDenoiseParameters.Backend] = [.reference, .nativeCPU]
        if NativeDenoiser.isGPUAvailable { backends.append(.nativeGPU) }
        return backends
    }

    func testDenoisePreservesVertexCountAndMovesVertices() async throws {
        for backend in Self.backendsUnderTest {
            let (positions, indices) = Self.noisyOctahedron()
            var params = MeshDenoiseParameters()
            params.backend = backend
            let result = try await MeshDenoiser.denoise(positions: positions, indices: indices, parameters: params)
            XCTAssertEqual(result.count, positions.count, "backend \(backend)")
            let maxDisplacement = zip(result, positions).map { length($0 - $1) }.max()!
            XCTAssertGreaterThan(maxDisplacement, 0, "backend \(backend)")
            XCTAssertLessThan(maxDisplacement, 1.0, "backend \(backend)")
        }
    }
```

(Add `import MeshDenoiserNative` to the test file for `NativeDenoiser.isGPUAvailable`. The `.nativeGPU` entries only become meaningful after Task 9 — until then `isGPUAvailable` is irrelevant because backendsUnderTest is only consulted at runtime; restrict to `[.reference, .nativeCPU]` in this task and add the GPU line in Task 9.)

In `GoldenParityTests.swift`, pin the strict 1e-6 test to the reference backend (it certifies the C++ wrapper, not the native port):

```swift
        var params = MeshDenoiseParameters() // defaults == CLI built-in defaults
        params.backend = .reference
        params.deterministic = true
```

- [ ] **Step 2: Run to verify failure**

Run: `swift test --filter NativeParityTests 2>&1 | tail -4`
Expected: FAIL — stub throws `.solverFailed` → `MeshDenoiseError.solverFailed`.

- [ ] **Step 3: Implement the orchestrator**

Replace `Sources/MeshDenoiserNative/NativeDenoiser.swift`:

```swift
import Metal
import simd

public enum NativeDenoiser {
    public static var isGPUAvailable: Bool {
        MetalFilterContext.shared != nil
    }

    /// progress returns false to cancel; called after each completed outer iteration.
    public static func denoise(
        positions: [SIMD3<Float>],
        indices: [UInt32],
        parameters: NativeDenoiseParameters,
        useGPU: Bool,
        progress: ((Int, Int) -> Bool)?
    ) throws -> [SIMD3<Float>] {
        guard parameters.isValid else { throw NativeDenoiseError.invalidParameters }
        guard !positions.isEmpty, !indices.isEmpty, indices.count % 3 == 0 else {
            throw NativeDenoiseError.invalidInput
        }
        for p in positions where !(p.x.isFinite && p.y.isFinite && p.z.isFinite) {
            throw NativeDenoiseError.invalidInput
        }
        for i in indices where i >= UInt32(positions.count) {
            throw NativeDenoiseError.invalidInput
        }

        var cancelled = false
        func isCancelled() -> Bool { cancelled }

        // Normalize: center at vertex mean, bbox diagonal → 1 (MeshTypes.h:208-220).
        var center = SIMD3<Double>.zero
        var lo = SIMD3<Double>(positions[0]), hi = lo
        for p in positions {
            let d = SIMD3<Double>(p)
            center += d
            lo = simd_min(lo, d); hi = simd_max(hi, d)
        }
        center /= Double(positions.count)
        let scale = simd_length(hi - lo)
        guard scale > 0 else { throw NativeDenoiseError.invalidInput }
        var current = positions.map { SIMD3<Float>((SIMD3<Double>($0) - center) / scale) }

        // Topology is constant: connectivity validation + the update factorization
        // happen once. Geometry-dependent quantities recompute per outer iteration.
        let topology = try MeshConnectivity(positions: current, indices: indices)
        let w = parameters.meshUpdateClosenessWeight
            * Double(topology.faces.count) / Double(topology.vertexCount)
        let updater = try VertexUpdater(faces: topology.faces,
                                        vertexCount: topology.vertexCount,
                                        closenessWeight: w)
        let gpu = useGPU ? MetalFilterContext.shared : nil

        var mesh = topology
        for outer in 1...parameters.outerIterations {
            let guidance = computeGuidanceNormals(mesh)
            let radius = Float(3 * parameters.eta * mesh.averageAdjacentCentroidDistance)
            let pairs = findNeighborPairs(centroids: mesh.faceCentroids, radius: radius)
            let operands = try precomputeFilterOperands(
                mesh: mesh, guidance: guidance, pairs: pairs, parameters: parameters)

            let targetNormals: [SIMD3<Float>]
            if let gpu {
                targetNormals = try gpu.run(initialNormals: mesh.faceNormals,
                                            areas: mesh.faceAreas,
                                            operands: operands, isCancelled: isCancelled)
            } else {
                targetNormals = try runFilterCPU(initialNormals: mesh.faceNormals,
                                                 areas: mesh.faceAreas,
                                                 operands: operands, isCancelled: isCancelled)
            }

            current = try updater.update(positions: current,
                                         targetNormals: targetNormals,
                                         iterations: parameters.meshUpdateIterations,
                                         displacementEps: parameters.meshUpdateDisplacementEps,
                                         isCancelled: isCancelled)

            if let progress, !progress(outer, parameters.outerIterations) {
                cancelled = true
                throw NativeDenoiseError.cancelled
            }
            if outer < parameters.outerIterations {
                mesh = try MeshConnectivity(positions: current, indices: indices)
            }
        }

        // Restore: re-center by current mean, then scale + original center (MeshTypes.h:224-236).
        var mean = SIMD3<Double>.zero
        for p in current { mean += SIMD3<Double>(p) }
        mean /= Double(current.count)
        return current.map { SIMD3<Float>((SIMD3<Double>($0) - mean) * scale + center) }
    }
}
```

Until Task 9 exists, add a temporary placeholder at the bottom of the file so it compiles (replaced in Task 9):

```swift
// Replaced by the real Metal implementation in FilterGPU.swift (Task 9).
final class MetalFilterContext {
    static let shared: MetalFilterContext? = nil
    func run(initialNormals: [SIMD3<Float>], areas: [Float],
             operands: FilterOperands, isCancelled: () -> Bool) throws -> [SIMD3<Float>] {
        throw NativeDenoiseError.solverFailed
    }
}
```

- [ ] **Step 4: Run the full suite**

Run: `swift test 2>&1 | grep -E "Executed .* tests|native-vs-golden"`
Expected: all PASS; note the printed `native-vs-golden: max=… mean=…` values. If max ≥ 2e-3, debug before loosening anything (likely suspects: λ rescale order, area normalization, guidance tie-breaks); use the per-stage tests to bisect.

- [ ] **Step 5: Commit (include measured parity numbers in the message)**

```bash
git add Sources/MeshDenoiserNative Tests/MeshDenoiserKitTests
git commit -m "Add native CPU denoiser pipeline with golden-tolerance parity

native-vs-golden on noisy_icosphere fixture: max=<value>, mean=<value>"
```

---

### Task 9: Metal filter

**Files:**
- Create: `Sources/MeshDenoiserNative/FilterKernels.metal`
- Create: `Sources/MeshDenoiserNative/FilterGPU.swift` (replaces the placeholder class in NativeDenoiser.swift — delete that)
- Create: `Tests/MeshDenoiserKitTests/GPUParityTests.swift`
- Modify: `Tests/MeshDenoiserKitTests/DenoiseContractTests.swift` (enable `.nativeGPU` in backendsUnderTest)

- [ ] **Step 1: Write the failing GPU parity test**

`Tests/MeshDenoiserKitTests/GPUParityTests.swift`:

```swift
import Metal
import simd
import XCTest
@testable import MeshDenoiserKit
@testable import MeshDenoiserNative

final class GPUParityTests: XCTestCase {

    func testGPUMatchesCPUOnFixture() async throws {
        try XCTSkipIf(!NativeDenoiser.isGPUAvailable, "No Metal device")
        let fixtures = Bundle.module.resourceURL!.appendingPathComponent("Fixtures")
        let input = try OBJLoader.load(fixtures.appendingPathComponent("noisy_icosphere.obj"))
        let positions = input.positions.map { SIMD3<Float>(Float($0.x), Float($0.y), Float($0.z)) }

        var cpu = MeshDenoiseParameters(); cpu.backend = .nativeCPU
        var gpu = MeshDenoiseParameters(); gpu.backend = .nativeGPU
        let a = try await MeshDenoiser.denoise(positions: positions, indices: input.indices, parameters: cpu)
        let b = try await MeshDenoiser.denoise(positions: positions, indices: input.indices, parameters: gpu)

        var maxE: Float = 0
        for (x, y) in zip(a, b) { maxE = max(maxE, simd_length(x - y)) }
        XCTAssertLessThan(maxE, 1e-4, "same float32 math; only reduction order differs")
    }

    func testNativeGPUWithoutDeviceThrows() async throws {
        try XCTSkipIf(NativeDenoiser.isGPUAvailable, "Only meaningful without a Metal device")
        var params = MeshDenoiseParameters(); params.backend = .nativeGPU
        do {
            _ = try await MeshDenoiser.denoise(
                positions: [[0,0,0], [1,0,0], [0,1,0]], indices: [0, 1, 2], parameters: params)
            XCTFail("Expected gpuUnavailable")
        } catch MeshDenoiseError.gpuUnavailable {}
    }
}
```

- [ ] **Step 2: Run to verify failure**

Run: `swift test --filter GPUParityTests 2>&1 | tail -3`
Expected: FAIL — `isGPUAvailable` is false (placeholder `shared = nil`), so the first test skips but the placeholder must now be replaced; after implementation the test runs for real. (On a machine without Metal both tests skip/pass trivially — implement anyway; CI macOS runners expose a Metal device.)

- [ ] **Step 3: Write the kernels**

`Sources/MeshDenoiserNative/FilterKernels.metal`:

```metal
#include <metal_stdlib>
using namespace metal;

struct FilterParams {
    uint pairCount;
    uint faceCount;
    float hNu;
    float pad;
};

kernel void computePairWeights(
    device const float4 *signals       [[buffer(0)]],
    device const uint2  *pairs         [[buffer(1)]],
    device const float  *staticWeights [[buffer(2)]],
    device float        *pairWeights   [[buffer(3)]],
    constant FilterParams &p           [[buffer(4)]],
    uint gid [[thread_position_in_grid]])
{
    if (gid >= p.pairCount) return;
    uint2 pr = pairs[gid];
    float3 d = signals[pr.x].xyz - signals[pr.y].xyz;
    pairWeights[gid] = staticWeights[gid] * exp(p.hNu * dot(d, d));
}

kernel void gatherAndNormalize(
    device const float4 *signals      [[buffer(0)]],
    device float4       *outSignals   [[buffer(1)]],
    device const float4 *weightedInit [[buffer(2)]],
    device const uint   *csrOffsets   [[buffer(3)]],
    device const uint2  *csrEntries   [[buffer(4)]],   // (neighbor face, pair index)
    device const float  *pairWeights  [[buffer(5)]],
    device const float  *areas        [[buffer(6)]],
    device float        *dispSq       [[buffer(7)]],
    constant FilterParams &p          [[buffer(8)]],
    uint gid [[thread_position_in_grid]])
{
    if (gid >= p.faceCount) return;
    float3 acc = weightedInit[gid].xyz;
    uint end = csrOffsets[gid + 1];
    for (uint k = csrOffsets[gid]; k < end; ++k) {
        uint2 e = csrEntries[k];
        acc += signals[e.x].xyz * pairWeights[e.y];
    }
    float len = length(acc);
    float3 n = (len > 0.0f) ? acc / len : float3(0.0f);
    outSignals[gid] = float4(n, 0.0f);
    float3 d = n - signals[gid].xyz;
    dispSq[gid] = areas[gid] * dot(d, d);
}

kernel void reduceSum256(
    device const float *input [[buffer(0)]],
    device float *partials    [[buffer(1)]],
    constant uint &count      [[buffer(2)]],
    uint gid   [[thread_position_in_grid]],
    uint lid   [[thread_position_in_threadgroup]],
    uint group [[threadgroup_position_in_grid]])
{
    threadgroup float shared[256];
    shared[lid] = (gid < count) ? input[gid] : 0.0f;
    threadgroup_barrier(mem_flags::mem_threadgroup);
    for (uint stride = 128; stride > 0; stride >>= 1) {
        if (lid < stride) shared[lid] += shared[lid + stride];
        threadgroup_barrier(mem_flags::mem_threadgroup);
    }
    if (lid == 0) partials[group] = shared[0];
}
```

- [ ] **Step 4: Implement the Metal host code**

Delete the placeholder `MetalFilterContext` class from `NativeDenoiser.swift`, then create `Sources/MeshDenoiserNative/FilterGPU.swift`:

```swift
import Metal
import simd

private struct FilterParamsGPU {
    var pairCount: UInt32
    var faceCount: UInt32
    var hNu: Float
    var pad: Float = 0
}

/// GPU fixed-point filter. One command buffer per iteration; convergence checked
/// on CPU from a 256-wide partial-sum reduction.
final class MetalFilterContext {
    static let shared: MetalFilterContext? = MetalFilterContext()

    private let device: MTLDevice
    private let queue: MTLCommandQueue
    private let pairWeightsPipeline: MTLComputePipelineState
    private let gatherPipeline: MTLComputePipelineState
    private let reducePipeline: MTLComputePipelineState

    private init?() {
        guard let device = MTLCreateSystemDefaultDevice(),
              let queue = device.makeCommandQueue(),
              let library = try? device.makeDefaultLibrary(bundle: Bundle.module),
              let f1 = library.makeFunction(name: "computePairWeights"),
              let f2 = library.makeFunction(name: "gatherAndNormalize"),
              let f3 = library.makeFunction(name: "reduceSum256"),
              let p1 = try? device.makeComputePipelineState(function: f1),
              let p2 = try? device.makeComputePipelineState(function: f2),
              let p3 = try? device.makeComputePipelineState(function: f3)
        else { return nil }
        self.device = device
        self.queue = queue
        self.pairWeightsPipeline = p1
        self.gatherPipeline = p2
        self.reducePipeline = p3
    }

    func run(
        initialNormals: [SIMD3<Float>],
        areas: [Float],
        operands: FilterOperands,
        isCancelled: () -> Bool
    ) throws -> [SIMD3<Float>] {
        let F = initialNormals.count
        let P = operands.pairFaces.count
        let groups = (F + 255) / 256

        func buffer<T>(_ array: [T]) throws -> MTLBuffer {
            guard let b = array.withUnsafeBytes({ bytes in
                device.makeBuffer(bytes: bytes.baseAddress!, length: bytes.count, options: .storageModeShared)
            }) else { throw NativeDenoiseError.solverFailed }
            return b
        }
        func empty(_ length: Int) throws -> MTLBuffer {
            guard let b = device.makeBuffer(length: max(length, 16), options: .storageModeShared) else {
                throw NativeDenoiseError.solverFailed
            }
            return b
        }

        let signals4 = initialNormals.map { SIMD4<Float>($0, 0) }
        let weightedInit4 = operands.weightedInit.map { SIMD4<Float>($0, 0) }
        var signalsA = try buffer(signals4)
        var signalsB = try empty(F * MemoryLayout<SIMD4<Float>>.stride)
        let pairsBuf = try buffer(operands.pairFaces)
        let staticBuf = try buffer(operands.staticWeights)
        let initBuf = try buffer(weightedInit4)
        let offsetsBuf = try buffer(operands.csrOffsets)
        let entriesBuf = try buffer(operands.csrEntries)
        let areasBuf = try buffer(areas)
        let pairWBuf = try empty(P * 4)
        let dispBuf = try empty(F * 4)
        let partialsBuf = try empty(groups * 4)
        var params = FilterParamsGPU(pairCount: UInt32(P), faceCount: UInt32(F), hNu: operands.hNu)
        var faceCount32 = UInt32(F)

        for _ in 1...operands.maxIterations {
            if isCancelled() { throw NativeDenoiseError.cancelled }
            guard let cb = queue.makeCommandBuffer(),
                  let enc = cb.makeComputeCommandEncoder() else {
                throw NativeDenoiseError.solverFailed
            }
            enc.setComputePipelineState(pairWeightsPipeline)
            enc.setBuffer(signalsA, offset: 0, index: 0)
            enc.setBuffer(pairsBuf, offset: 0, index: 1)
            enc.setBuffer(staticBuf, offset: 0, index: 2)
            enc.setBuffer(pairWBuf, offset: 0, index: 3)
            enc.setBytes(&params, length: MemoryLayout<FilterParamsGPU>.stride, index: 4)
            enc.dispatchThreadgroups(MTLSize(width: (P + 255) / 256, height: 1, depth: 1),
                                     threadsPerThreadgroup: MTLSize(width: 256, height: 1, depth: 1))

            enc.setComputePipelineState(gatherPipeline)
            enc.setBuffer(signalsA, offset: 0, index: 0)
            enc.setBuffer(signalsB, offset: 0, index: 1)
            enc.setBuffer(initBuf, offset: 0, index: 2)
            enc.setBuffer(offsetsBuf, offset: 0, index: 3)
            enc.setBuffer(entriesBuf, offset: 0, index: 4)
            enc.setBuffer(pairWBuf, offset: 0, index: 5)
            enc.setBuffer(areasBuf, offset: 0, index: 6)
            enc.setBuffer(dispBuf, offset: 0, index: 7)
            enc.setBytes(&params, length: MemoryLayout<FilterParamsGPU>.stride, index: 8)
            enc.dispatchThreadgroups(MTLSize(width: groups, height: 1, depth: 1),
                                     threadsPerThreadgroup: MTLSize(width: 256, height: 1, depth: 1))

            enc.setComputePipelineState(reducePipeline)
            enc.setBuffer(dispBuf, offset: 0, index: 0)
            enc.setBuffer(partialsBuf, offset: 0, index: 1)
            enc.setBytes(&faceCount32, length: 4, index: 2)
            enc.dispatchThreadgroups(MTLSize(width: groups, height: 1, depth: 1),
                                     threadsPerThreadgroup: MTLSize(width: 256, height: 1, depth: 1))
            enc.endEncoding()
            cb.commit()
            cb.waitUntilCompleted()
            guard cb.status == .completed else { throw NativeDenoiseError.solverFailed }

            swap(&signalsA, &signalsB)

            var disp = 0.0
            let partials = partialsBuf.contents().bindMemory(to: Float.self, capacity: groups)
            for g in 0..<groups { disp += Double(partials[g]) }
            if disp <= operands.convergenceThreshold { break }
        }

        let out = signalsA.contents().bindMemory(to: SIMD4<Float>.self, capacity: F)
        return (0..<F).map { SIMD3(out[$0].x, out[$0].y, out[$0].z) }
    }
}
```

Enable the GPU backend in `DenoiseContractTests.backendsUnderTest` (add the `if NativeDenoiser.isGPUAvailable { backends.append(.nativeGPU) }` line if deferred from Task 8).

- [ ] **Step 5: Run the full suite**

Run: `swift test 2>&1 | grep -E "Executed .* tests|passed|failed" | tail -4`
Expected: all PASS, GPU tests included on this machine. Troubleshooting: if `makeDefaultLibrary(bundle: Bundle.module)` fails, the `.metal` file wasn't compiled into the module bundle — check it sits inside `Sources/MeshDenoiserNative/` (SwiftPM compiles Metal sources into `default.metallib` automatically; if the build system used doesn't, add `resources: [.process("FilterKernels.metal")]` to the target as fallback).

- [ ] **Step 6: Commit**

```bash
git add Sources/MeshDenoiserNative Tests/MeshDenoiserKitTests
git commit -m "Add Metal fixed-point filter with CPU-parity test"
```

---

### Task 10: Benchmark tool + recorded results

**Files:**
- Create: `Sources/MeshDenoiserBench/main.swift`
- Modify: `Package.swift` (executable target + product)
- Create: `docs/benchmarks.md`

- [ ] **Step 1: Add the executable target**

In `Package.swift` products add:

```swift
        .executable(name: "meshdenoiser-bench", targets: ["MeshDenoiserBench"]),
```

and targets:

```swift
        .executableTarget(
            name: "MeshDenoiserBench",
            dependencies: ["MeshDenoiserKit", "MeshDenoiserNative"],
            path: "Sources/MeshDenoiserBench"
        ),
```

- [ ] **Step 2: Write the tool**

`Sources/MeshDenoiserBench/main.swift`:

```swift
import Foundation
import MeshDenoiserKit
import MeshDenoiserNative

func icosphere(subdivisions: Int, noise: Float) -> (positions: [SIMD3<Float>], indices: [UInt32]) {
    let phi = Float((1 + 5.0.squareRoot()) / 2)
    var verts: [SIMD3<Float>] = [
        [-1, phi, 0], [1, phi, 0], [-1, -phi, 0], [1, -phi, 0],
        [0, -1, phi], [0, 1, phi], [0, -1, -phi], [0, 1, -phi],
        [phi, 0, -1], [phi, 0, 1], [-phi, 0, -1], [-phi, 0, 1],
    ].map(simd_normalize)
    var faces: [SIMD3<UInt32>] = [
        [0,11,5],[0,5,1],[0,1,7],[0,7,10],[0,10,11],[1,5,9],[5,11,4],[11,10,2],[10,7,6],[7,1,8],
        [3,9,4],[3,4,2],[3,2,6],[3,6,8],[3,8,9],[4,9,5],[2,4,11],[6,2,10],[8,6,7],[9,8,1],
    ]
    for _ in 0..<subdivisions {
        var cache = [UInt64: UInt32]()
        func mid(_ a: UInt32, _ b: UInt32) -> UInt32 {
            let key = UInt64(min(a, b)) << 32 | UInt64(max(a, b))
            if let m = cache[key] { return m }
            verts.append(simd_normalize((verts[Int(a)] + verts[Int(b)]) / 2))
            cache[key] = UInt32(verts.count - 1)
            return cache[key]!
        }
        var next: [SIMD3<UInt32>] = []
        next.reserveCapacity(faces.count * 4)
        for f in faces {
            let ab = mid(f.x, f.y), bc = mid(f.y, f.z), ca = mid(f.z, f.x)
            next += [[f.x, ab, ca], [f.y, bc, ab], [f.z, ca, bc], [ab, bc, ca]]
        }
        faces = next
    }
    var state: UInt64 = 12345
    func rand() -> Float {
        state = (state &* 1103515245 &+ 12345) % (1 << 31)
        return Float(state) / Float(1 << 31)
    }
    let noisy = verts.map { $0 * (1 + (rand() - 0.5) * noise) }
    return (noisy, faces.flatMap { [$0.x, $0.y, $0.z] })
}

let levels = CommandLine.arguments.dropFirst().compactMap { Int($0) }
let runLevels = levels.isEmpty ? [5, 6, 7] : levels

print("| faces | reference | nativeCPU | nativeGPU |")
print("|---|---|---|---|")
for level in runLevels {
    let (positions, indices) = icosphere(subdivisions: level, noise: 0.1)
    var row = "| \(indices.count / 3) |"
    for backend in [MeshDenoiseParameters.Backend.reference, .nativeCPU, .nativeGPU] {
        if backend == .nativeGPU && !NativeDenoiser.isGPUAvailable { row += " n/a |"; continue }
        var params = MeshDenoiseParameters()
        params.backend = backend
        let start = Date()
        let semaphore = DispatchSemaphore(value: 0)
        var elapsed = "fail"
        Task {
            do {
                _ = try await MeshDenoiser.denoise(positions: positions, indices: indices, parameters: params)
                elapsed = String(format: "%.2fs", Date().timeIntervalSince(start))
            } catch { elapsed = "error: \(error)" }
            semaphore.signal()
        }
        semaphore.wait()
        row += " \(elapsed) |"
    }
    print(row)
}
```

- [ ] **Step 3: Run and record**

Run: `swift run -c release meshdenoiser-bench 5 6 7`
Expected: a markdown table; nativeCPU and nativeGPU substantially faster than reference at level 7 (target from spec: ≥5× total at 328k faces). Save the output:

Create `docs/benchmarks.md` with the produced table plus the machine description and date, and the baseline C++ stage breakdown from the spec.

- [ ] **Step 4: Commit**

```bash
git add Package.swift Sources/MeshDenoiserBench docs/benchmarks.md
git commit -m "Add benchmark tool and record native-vs-reference results"
```

---

### Task 11: Flip the default backend + docs + final verification

**Files:**
- Modify: `Sources/MeshDenoiserKit/MeshDenoiseParameters.swift` (default `.automatic`)
- Modify: `README.md`

- [ ] **Step 1: Flip the default**

```swift
    public var backend: Backend = .automatic
```

- [ ] **Step 2: Full clean verification**

Run: `swift package clean && swift test 2>&1 | grep -E "Executed .* tests" | tail -1`
Expected: all tests pass — contract tests now exercise native by default; `GoldenParityTests` still pins `.reference` explicitly.

- [ ] **Step 3: Update README**

In the "Using from Swift (iOS / macOS)" section of `README.md`, after the code block, add:

```markdown
The denoiser runs natively in Swift + Metal by default (`backend: .automatic`,
GPU with CPU fallback). `backend: .reference` selects the wrapped C++
implementation, kept temporarily as a validation oracle. The native backend
always uses Accelerate's sparse Cholesky for the vertex update; the
`linearSolver` option only affects the reference backend. See
`docs/benchmarks.md` for performance numbers.
```

- [ ] **Step 4: Commit**

```bash
git add Sources/MeshDenoiserKit/MeshDenoiseParameters.swift README.md
git commit -m "Default to native Swift/Metal backend"
```

- [ ] **Step 5: Working-tree check**

Run: `git status --short`
Expected: only the pre-existing user WIP (`src/MeshDenoiser.cpp` --gui changes, `meshdenoiser_gui.py`, `test.usdz`).

---

## Follow-ups (explicitly out of this plan)

- On-device validation in Plinth (iPhone/iPad), then a removal PR for `CMeshDenoiserCore` + vendored Eigen/OpenMesh.
- GPU memory tuning: batch multiple filter iterations per command buffer if profiling shows readback dominating.
- iOS memory ceiling test at 1M+ faces; consider streaming pair generation if the pair list itself becomes the limit.
