import simd
import XCTest
@testable import MeshDenoiserKit
@testable import MeshDenoiserNative

final class GPUParityTests: XCTestCase {
    func testFaceNeighborRowsUseMetalUInt2Layout() {
        XCTAssertTrue(FilterGPU.faceNeighborRowsUseMetalUInt2LayoutForTesting())
    }

    func testMetalDisplacementReductionMatchesCPUFormula() throws {
        guard FilterGPU.isAvailable else {
            throw XCTSkip("Metal is not available on this host")
        }

        let previous: [SIMD3<Float>] = [
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 1],
            [-1, 0, 0],
        ]
        let current: [SIMD3<Float>] = [
            [0.9, 0.1, 0],
            [0.1, 0.95, 0],
            [0, 0.1, 0.9],
            [-0.95, 0.05, 0],
        ]
        let areaWeights: [Float] = [1.0, 0.75, 1.25, 2.0]

        var expected: Float = 0
        for index in previous.indices {
            let diff = current[index] - previous[index]
            expected += areaWeights[index] * (diff.x * diff.x + diff.y * diff.y + diff.z * diff.z)
        }

        let got = try FilterGPU.displacementForTesting(
            previous: previous,
            current: current,
            areaWeights: areaWeights
        )

        XCTAssertEqual(got, expected, accuracy: 1e-6)
    }

    func testMetalFilterMatchesCPUFilterOnOctahedron() throws {
        guard FilterGPU.isAvailable else {
            throw XCTSkip("Metal is not available on this host")
        }

        let meshData = DenoiseContractTests.noisyOctahedron()
        var input = try makeFilterInput(positions: meshData.positions, indices: meshData.indices)

        let cpu = try FilterCPU.run(
            initialSignals: input.connectivity.faceGeometry.normals,
            areaWeights: input.connectivity.faceGeometry.areaWeights,
            precompute: input.precompute,
            nu: Float(NativeDenoiseParameters().nu)
        )
        let gpu = try FilterGPU.run(
            initialSignals: input.connectivity.faceGeometry.normals,
            areaWeights: input.connectivity.faceGeometry.areaWeights,
            precompute: &input.precompute,
            nu: Float(NativeDenoiseParameters().nu)
        )

        XCTAssertEqual(gpu.iterations, cpu.iterations)
        XCTAssertEqual(gpu.converged, cpu.converged)
        for (got, want) in zip(gpu.signals, cpu.signals) {
            XCTAssertEqual(got.x, want.x, accuracy: 2e-6)
            XCTAssertEqual(got.y, want.y, accuracy: 2e-6)
            XCTAssertEqual(got.z, want.z, accuracy: 2e-6)
        }
    }

    func testMetalFilterMatchesCPUFilterOnFixture() throws {
        guard FilterGPU.isAvailable else {
            throw XCTSkip("Metal is not available on this host")
        }

        let fixtures = Bundle.module.resourceURL!.appendingPathComponent("Fixtures")
        let fixture = try OBJLoader.load(fixtures.appendingPathComponent("noisy_icosphere.obj"))
        let positions = fixture.positions.map { SIMD3<Float>(Float($0.x), Float($0.y), Float($0.z)) }
        var input = try makeFilterInput(positions: positions, indices: fixture.indices)

        let cpu = try FilterCPU.run(
            initialSignals: input.connectivity.faceGeometry.normals,
            areaWeights: input.connectivity.faceGeometry.areaWeights,
            precompute: input.precompute,
            nu: Float(NativeDenoiseParameters().nu)
        )
        let gpu = try FilterGPU.run(
            initialSignals: input.connectivity.faceGeometry.normals,
            areaWeights: input.connectivity.faceGeometry.areaWeights,
            precompute: &input.precompute,
            nu: Float(NativeDenoiseParameters().nu)
        )

        var maxNormalError: Float = 0
        for (got, want) in zip(gpu.signals, cpu.signals) {
            maxNormalError = max(maxNormalError, simd.length(got - want))
        }

        XCTAssertLessThan(maxNormalError, 1e-4)
        XCTAssertTrue(input.precompute.pairs.isEmpty)
        XCTAssertTrue(input.precompute.staticWeights.isEmpty)
        XCTAssertTrue(input.precompute.weightedInitialSignals.isEmpty)
        XCTAssertTrue(input.precompute.faceNeighborRows.offsets.isEmpty)
        XCTAssertTrue(input.precompute.faceNeighborRows.values.isEmpty)
    }

    func testMetalCancellationStopsBeforeConsumingPrecompute() throws {
        guard FilterGPU.isAvailable else {
            throw XCTSkip("Metal is not available on this host")
        }

        let meshData = DenoiseContractTests.noisyOctahedron()
        var input = try makeFilterInput(positions: meshData.positions, indices: meshData.indices)
        let originalPairCount = input.precompute.pairs.count

        XCTAssertThrowsError(try FilterGPU.run(
            initialSignals: input.connectivity.faceGeometry.normals,
            areaWeights: input.connectivity.faceGeometry.areaWeights,
            precompute: &input.precompute,
            nu: Float(NativeDenoiseParameters().nu),
            shouldCancel: { true }
        )) { error in
            XCTAssertEqual(error as? NativeDenoiseError, .cancelled)
        }
        XCTAssertEqual(input.precompute.pairs.count, originalPairCount)
    }

    private func makeFilterInput(
        positions: [SIMD3<Float>],
        indices: [UInt32]
    ) throws -> (connectivity: MeshConnectivity.Result, precompute: FilterPrecompute) {
        let mesh = try MeshValidation.makeMesh(positions: positions, indices: indices)
        let connectivity = try MeshConnectivity.build(mesh: mesh)
        let guidance = GuidanceNormals.compute(connectivity: connectivity)
        let etaPrime = Float(NativeDenoiseParameters().eta)
            * connectivity.averageNeighborFaceCentroidDistance
        let pairs = try NeighborSearch.findPairs(
            centroids: connectivity.faceGeometry.centroids,
            radius: 3 * etaPrime
        )
        let precompute = try FilterPrecompute.build(
            pairs: pairs,
            guidanceNormals: guidance,
            initialNormals: connectivity.faceGeometry.normals,
            areaWeights: connectivity.faceGeometry.areaWeights,
            etaPrime: etaPrime,
            parameters: NativeDenoiseParameters()
        )
        return (connectivity, precompute)
    }
}
