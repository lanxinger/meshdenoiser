import XCTest
@testable import MeshDenoiserNative

final class FilterCPUTests: XCTestCase {

    func testIdenticalSignalsConvergeInOneIteration() throws {
        let initialSignals = [SIMD3<Float>(0, 0, 1), SIMD3<Float>(0, 0, 1)]
        let areaWeights: [Float] = [1, 1]
        let pairs = [NeighborPair(face0: 0, face1: 1, distance: 0.25)]
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
        let pairs = [NeighborPair(face0: 0, face1: 1, distance: 0.25)]
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

    func testCancellationStopsBeforeFiltering() throws {
        let initialSignals = [SIMD3<Float>(0, 0, 1), SIMD3<Float>(0, 1, 0)]
        let areaWeights: [Float] = [1, 1]
        let params = NativeDenoiseParameters()
        let precompute = try FilterPrecompute.build(
            pairs: [NeighborPair(face0: 0, face1: 1, distance: 0.25)],
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
}
