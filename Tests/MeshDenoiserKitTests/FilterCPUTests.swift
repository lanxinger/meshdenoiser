import XCTest
@testable import MeshDenoiserNative

final class FilterCPUTests: XCTestCase {

    func testIdenticalSignalsConvergeInOneIteration() throws {
        let initialSignals = [SIMD3<Float>(0, 0, 1), SIMD3<Float>(0, 0, 1)]
        let areaWeights: [Float] = [1, 1]
        let pairs = [NeighborPair(face0: 0, face1: 1, distanceSquared: 0.0625)]
        let params = NativeDenoiseParameters()
        let precompute = try FilterPrecompute.build(
            pairs: pairs,
            guidanceNormals: initialSignals,
            initialNormals: initialSignals,
            areaWeights: areaWeights,
            etaPrime: 1,
            parameters: params
        )

        let result = try FilterCPU.run(
            initialSignals: initialSignals,
            areaWeights: areaWeights,
            precompute: precompute,
            nu: Float(params.nu)
        )

        XCTAssertTrue(result.converged)
        XCTAssertEqual(result.iterations, 1)
        XCTAssertEqual(result.signals[0].z, 1, accuracy: 1e-6)
        XCTAssertEqual(result.signals[1].z, 1, accuracy: 1e-6)
    }

    func testFilterDoesNotRequireFaceNeighborRows() throws {
        let initialSignals = [SIMD3<Float>(0, 0, 1), SIMD3<Float>(0, 1, 0)]
        let areaWeights: [Float] = [1, 1]
        let pairs = [NeighborPair(face0: 0, face1: 1, distanceSquared: 0.0625)]
        let params = NativeDenoiseParameters()
        var precompute = try FilterPrecompute.build(
            pairs: pairs,
            guidanceNormals: initialSignals,
            initialNormals: initialSignals,
            areaWeights: areaWeights,
            etaPrime: 1,
            parameters: params
        )
        precompute.faceNeighborRows = CSRAdjacency(offsets: [], values: [])

        let result = try FilterCPU.run(
            initialSignals: initialSignals,
            areaWeights: areaWeights,
            precompute: precompute,
            nu: Float(params.nu),
            maxIterations: 2
        )

        XCTAssertEqual(result.signals.count, initialSignals.count)
        XCTAssertTrue(result.signals.allSatisfy { $0.x.isFinite && $0.y.isFinite && $0.z.isFinite })
    }

    /// The parallel CSR gather must reproduce a serial pair-order scatter bit for bit,
    /// on a mesh large enough to span many work chunks, and be repeatable.
    func testParallelGatherMatchesSerialPairScatterAcrossChunks() throws {
        let sphere = Self.noisySphere(rings: 30, segments: 60)
        let mesh = try MeshValidation.makeMesh(positions: sphere.positions, indices: sphere.indices)
        let connectivity = try MeshConnectivity.build(mesh: mesh)
        XCTAssertGreaterThan(connectivity.faceGeometry.normals.count, 4 * 512)
        let guidance = GuidanceNormals.compute(connectivity: connectivity)
        let params = NativeDenoiseParameters()
        let etaPrime = Float(params.eta) * connectivity.averageNeighborFaceCentroidDistance

        let streaming = try FilterPrecompute.buildForCPU(
            centroids: connectivity.faceGeometry.centroids,
            guidanceNormals: guidance,
            initialNormals: connectivity.faceGeometry.normals,
            areaWeights: connectivity.faceGeometry.areaWeights,
            etaPrime: etaPrime,
            parameters: params
        )
        let materialized = try FilterPrecompute.build(
            centroids: connectivity.faceGeometry.centroids,
            guidanceNormals: guidance,
            initialNormals: connectivity.faceGeometry.normals,
            areaWeights: connectivity.faceGeometry.areaWeights,
            etaPrime: etaPrime,
            parameters: params
        )
        XCTAssertTrue(streaming.pairs.isEmpty)

        let gathered = try FilterCPU.run(
            initialSignals: connectivity.faceGeometry.normals,
            areaWeights: connectivity.faceGeometry.areaWeights,
            precompute: streaming,
            nu: Float(params.nu)
        )
        let repeated = try FilterCPU.run(
            initialSignals: connectivity.faceGeometry.normals,
            areaWeights: connectivity.faceGeometry.areaWeights,
            precompute: streaming,
            nu: Float(params.nu)
        )
        let scattered = Self.serialPairScatter(
            initialSignals: connectivity.faceGeometry.normals,
            areaWeights: connectivity.faceGeometry.areaWeights,
            precompute: materialized,
            nu: Float(params.nu),
            iterations: gathered.iterations
        )

        XCTAssertEqual(gathered.signals, scattered)
        XCTAssertEqual(repeated.signals, gathered.signals)
        XCTAssertEqual(repeated.iterations, gathered.iterations)
    }

    func testCancellationStopsBeforeFiltering() throws {
        let initialSignals = [SIMD3<Float>(0, 0, 1), SIMD3<Float>(0, 1, 0)]
        let areaWeights: [Float] = [1, 1]
        let params = NativeDenoiseParameters()
        let precompute = try FilterPrecompute.build(
            pairs: [NeighborPair(face0: 0, face1: 1, distanceSquared: 0.0625)],
            guidanceNormals: initialSignals,
            initialNormals: initialSignals,
            areaWeights: areaWeights,
            etaPrime: 1,
            parameters: params
        )

        XCTAssertThrowsError(try FilterCPU.run(
            initialSignals: initialSignals,
            areaWeights: areaWeights,
            precompute: precompute,
            nu: Float(params.nu),
            shouldCancel: { true }
        )) { error in
            XCTAssertEqual(error as? NativeDenoiseError, .cancelled)
        }
    }

    /// Single-threaded pair-order scatter (the pre-parallel formulation).
    private static func serialPairScatter(
        initialSignals: [SIMD3<Float>],
        areaWeights: [Float],
        precompute: FilterPrecompute,
        nu: Float,
        iterations: Int
    ) -> [SIMD3<Float>] {
        var signals = initialSignals
        let h = Float(-0.5) / (nu * nu)
        for _ in 0..<iterations {
            var filtered = precompute.weightedInitialSignals
            for (pairIndex, pair) in precompute.pairs.enumerated() {
                let face0 = Int(pair.face0)
                let face1 = Int(pair.face1)
                let diff = signals[face0] - signals[face1]
                let weight = precompute.staticWeights[pairIndex] * Float(exp(Double(h * lengthSquared(diff))))
                filtered[face0] += signals[face1] * weight
                filtered[face1] += signals[face0] * weight
            }
            for face in filtered.indices {
                filtered[face] = normalized(filtered[face], fallback: signals[face])
            }
            signals = filtered
        }
        return signals
    }

    /// UV sphere with deterministic radial noise.
    static func noisySphere(rings: Int, segments: Int) -> (positions: [SIMD3<Float>], indices: [UInt32]) {
        var state: UInt64 = 0x9E37_79B9_7F4A_7C15
        func noise() -> Float {
            state = state &* 6_364_136_223_846_793_005 &+ 1_442_695_040_888_963_407
            return (Float(state >> 40) / Float(1 << 24) - 0.5) * 0.01
        }

        var positions: [SIMD3<Float>] = [[0, 0, 1]]
        for ring in 1..<rings {
            let theta = Float.pi * Float(ring) / Float(rings)
            for segment in 0..<segments {
                let phi = 2 * Float.pi * Float(segment) / Float(segments)
                let radius = 1 + noise()
                positions.append(SIMD3(
                    sin(theta) * cos(phi) * radius,
                    sin(theta) * sin(phi) * radius,
                    cos(theta) * radius
                ))
            }
        }
        positions.append([0, 0, -1])
        let south = UInt32(positions.count - 1)
        func vertex(_ ring: Int, _ segment: Int) -> UInt32 {
            UInt32(1 + (ring - 1) * segments + segment % segments)
        }

        var indices: [UInt32] = []
        for segment in 0..<segments {
            indices += [0, vertex(1, segment), vertex(1, segment + 1)]
        }
        for ring in 1..<(rings - 1) {
            for segment in 0..<segments {
                indices += [vertex(ring, segment), vertex(ring + 1, segment), vertex(ring + 1, segment + 1)]
                indices += [vertex(ring, segment), vertex(ring + 1, segment + 1), vertex(ring, segment + 1)]
            }
        }
        for segment in 0..<segments {
            indices += [vertex(rings - 1, segment), south, vertex(rings - 1, segment + 1)]
        }
        return (positions, indices)
    }
}
