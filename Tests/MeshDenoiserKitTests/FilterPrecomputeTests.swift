import XCTest
@testable import MeshDenoiserKit
@testable import MeshDenoiserNative

final class FilterPrecomputeTests: XCTestCase {

    func testStaticWeightsAndLambdaRescaleMatchScalarFormula() throws {
        let pairs = [NeighborPair(face0: 0, face1: 1, distance: 0.5)]
        let guidance: [SIMD3<Float>] = [[0, 0, 1], [0, 1, 0]]
        let initial: [SIMD3<Float>] = [[0, 0, 1], [0, 1, 0]]
        let areaWeights: [Float] = [1.5, 0.5]
        var params = NativeDenoiseParameters()
        params.lambda = 0.15
        params.eta = 2.2
        params.mu = 0.2
        params.nu = 0.25
        let etaPrime: Float = 1.25

        let precompute = try FilterPrecompute.build(
            pairs: pairs,
            guidanceNormals: guidance,
            initialNormals: initial,
            areaWeights: areaWeights,
            etaPrime: etaPrime,
            parameters: params
        )

        let hSpatial = Float(-0.5) / (etaPrime * etaPrime)
        let hGuidance = Float(-0.5) / Float(params.mu * params.mu)
        let areaSum = areaWeights[0] + areaWeights[1]
        let areaSpatial = areaSum * exp(hSpatial * pairs[0].distance * pairs[0].distance)
        let expectedStatic = areaSum * exp(
            hGuidance * lengthSquared(guidance[0] - guidance[1])
                + hSpatial * pairs[0].distance * pairs[0].distance
        )
        let expectedLambda = Float(params.lambda) * areaWeights.reduce(0, +) / areaSpatial
        let expectedInitScale = Float(2 * params.nu * params.nu) / expectedLambda

        XCTAssertEqual(precompute.staticWeights[0], expectedStatic, accuracy: 1e-6)
        XCTAssertEqual(precompute.rescaledLambda, expectedLambda, accuracy: 1e-6)
        XCTAssertEqual(precompute.weightedInitialSignals[0].z, initial[0].z * areaWeights[0] * expectedInitScale, accuracy: 1e-6)
        XCTAssertEqual(precompute.faceNeighborRows.values.count, 2)
        XCTAssertEqual(precompute.faceNeighborRows.values[0], FaceNeighbor(neighborFace: 1, pairIndex: 0))
        XCTAssertEqual(precompute.faceNeighborRows.values[1], FaceNeighbor(neighborFace: 0, pairIndex: 0))
    }

    func testStreamingBuildMatchesMaterializedPairs() throws {
        let meshData = DenoiseContractTests.noisyOctahedron()
        let mesh = try MeshValidation.makeMesh(positions: meshData.positions, indices: meshData.indices)
        let connectivity = try MeshConnectivity.build(mesh: mesh)
        let guidance = GuidanceNormals.compute(connectivity: connectivity)
        let etaPrime = Float(NativeDenoiseParameters().eta)
            * connectivity.averageNeighborFaceCentroidDistance
        let pairs = try NeighborSearch.findPairs(
            centroids: connectivity.faceGeometry.centroids,
            radius: 3 * etaPrime
        )

        let materialized = try FilterPrecompute.build(
            pairs: pairs,
            guidanceNormals: guidance,
            initialNormals: connectivity.faceGeometry.normals,
            areaWeights: connectivity.faceGeometry.areaWeights,
            etaPrime: etaPrime,
            parameters: NativeDenoiseParameters()
        )
        let streaming = try FilterPrecompute.build(
            centroids: connectivity.faceGeometry.centroids,
            guidanceNormals: guidance,
            initialNormals: connectivity.faceGeometry.normals,
            areaWeights: connectivity.faceGeometry.areaWeights,
            etaPrime: etaPrime,
            parameters: NativeDenoiseParameters()
        )

        XCTAssertEqual(streaming.pairs, materialized.pairs)
        XCTAssertEqual(streaming.staticWeights, materialized.staticWeights)
        XCTAssertEqual(streaming.faceNeighborRows.offsets, materialized.faceNeighborRows.offsets)
        XCTAssertEqual(streaming.faceNeighborRows.values, materialized.faceNeighborRows.values)
        XCTAssertEqual(streaming.weightedInitialSignals, materialized.weightedInitialSignals)
        XCTAssertEqual(streaming.rescaledLambda, materialized.rescaledLambda)
    }

    func testGPUBuildAvoidsStoredPairsButMatchesRowsAndWeights() throws {
        let meshData = DenoiseContractTests.noisyOctahedron()
        let mesh = try MeshValidation.makeMesh(positions: meshData.positions, indices: meshData.indices)
        let connectivity = try MeshConnectivity.build(mesh: mesh)
        let guidance = GuidanceNormals.compute(connectivity: connectivity)
        let etaPrime = Float(NativeDenoiseParameters().eta)
            * connectivity.averageNeighborFaceCentroidDistance

        let cpu = try FilterPrecompute.build(
            centroids: connectivity.faceGeometry.centroids,
            guidanceNormals: guidance,
            initialNormals: connectivity.faceGeometry.normals,
            areaWeights: connectivity.faceGeometry.areaWeights,
            etaPrime: etaPrime,
            parameters: NativeDenoiseParameters()
        )
        let gpu = try FilterPrecompute.buildForGPU(
            centroids: connectivity.faceGeometry.centroids,
            guidanceNormals: guidance,
            initialNormals: connectivity.faceGeometry.normals,
            areaWeights: connectivity.faceGeometry.areaWeights,
            etaPrime: etaPrime,
            parameters: NativeDenoiseParameters()
        )

        XCTAssertTrue(gpu.pairs.isEmpty)
        XCTAssertEqual(gpu.staticWeights, cpu.staticWeights)
        XCTAssertEqual(gpu.faceNeighborRows.offsets, cpu.faceNeighborRows.offsets)
        XCTAssertEqual(gpu.faceNeighborRows.values, cpu.faceNeighborRows.values)
        XCTAssertEqual(gpu.weightedInitialSignals, cpu.weightedInitialSignals)
        XCTAssertEqual(gpu.rescaledLambda, cpu.rescaledLambda)
    }
}
