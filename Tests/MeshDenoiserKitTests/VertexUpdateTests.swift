import XCTest
@testable import MeshDenoiserKit
@testable import MeshDenoiserNative

final class VertexUpdateTests: XCTestCase {

    func testSystemMatrixSingleTriangleBlockMatchesMeanCenteringPlusCloseness() {
        let triangles = [SIMD3<UInt32>(0, 1, 2)]
        let matrix = VertexUpdate.systemMatrixForTesting(
            vertexCount: 3,
            triangles: triangles,
            closenessWeight: 0.25
        )

        XCTAssertEqual(matrix[0][0], 2.0 / 3.0 + 0.25, accuracy: 1e-12)
        XCTAssertEqual(matrix[1][1], 2.0 / 3.0 + 0.25, accuracy: 1e-12)
        XCTAssertEqual(matrix[2][2], 2.0 / 3.0 + 0.25, accuracy: 1e-12)
        XCTAssertEqual(matrix[0][1], -1.0 / 3.0, accuracy: 1e-12)
        XCTAssertEqual(matrix[0][2], -1.0 / 3.0, accuracy: 1e-12)
        XCTAssertEqual(matrix[1][2], -1.0 / 3.0, accuracy: 1e-12)
    }

    func testPlaneProjectionPreservesCenteredTriangleCentroid() {
        let centered = [
            SIMD3<Double>(-1, -1, 0.25),
            SIMD3<Double>(1, 0, -0.5),
            SIMD3<Double>(0, 1, 0.25),
        ]
        let projected = VertexUpdate.projectToTargetPlaneForTesting(
            centeredPositions: centered,
            targetNormal: SIMD3<Double>(0, 0, 1)
        )

        let centroid = projected.reduce(SIMD3<Double>(repeating: 0), +) / 3
        XCTAssertEqual(centroid.x, 0, accuracy: 1e-12)
        XCTAssertEqual(centroid.y, 0, accuracy: 1e-12)
        XCTAssertEqual(centroid.z, 0, accuracy: 1e-12)
        for point in projected {
            XCTAssertEqual(point.z, 0, accuracy: 1e-12)
        }
    }

    func testCompleteUpdatePassProducesFinitePositionsAndPreservesCount() throws {
        let meshData = DenoiseContractTests.noisyOctahedron()
        let mesh = try MeshValidation.makeMesh(positions: meshData.positions, indices: meshData.indices)
        let connectivity = try MeshConnectivity.build(mesh: mesh)
        var params = NativeDenoiseParameters()
        params.meshUpdateIterations = 2

        let updated = try VertexUpdate.run(
            mesh: mesh,
            targetNormals: connectivity.faceGeometry.normals,
            parameters: params
        )

        XCTAssertEqual(updated.count, mesh.positions.count)
        for position in updated {
            XCTAssertTrue(position.x.isFinite)
            XCTAssertTrue(position.y.isFinite)
            XCTAssertTrue(position.z.isFinite)
        }
    }

    /// The three-column, Jacobi-preconditioned, warm-started CG must solve
    /// (A^T A + wI) x = b for each column, on a mesh spanning several work chunks.
    func testSolverSatisfiesSystemAcrossChunksFromColdAndWarmStarts() throws {
        let grid = Self.grid(width: 70, height: 60)
        XCTAssertGreaterThan(grid.vertexCount, 2 * 2_048)
        let closenessWeight = 0.001 * Double(grid.triangles.count) / Double(grid.vertexCount)

        var state: UInt64 = 42
        func random() -> Double {
            state = state &* 6_364_136_223_846_793_005 &+ 1_442_695_040_888_963_407
            return Double(state >> 11) / Double(1 << 53) - 0.5
        }
        let rhs = (0..<grid.vertexCount).map { _ in SIMD3<Double>(random(), random(), random()) * 1e-3 }
        let warmGuess = (0..<grid.vertexCount).map { _ in SIMD3<Double>(random(), random(), random()) }

        for initialGuess in [[SIMD3<Double>](repeating: .zero, count: grid.vertexCount), warmGuess] {
            let solution = try VertexUpdate.solveForTesting(
                triangles: grid.triangles,
                vertexCount: grid.vertexCount,
                closenessWeight: closenessWeight,
                rhs: rhs,
                initialGuess: initialGuess
            )
            let product = Self.applySystem(solution, triangles: grid.triangles, closenessWeight: closenessWeight)
            var residual = SIMD3<Double>.zero
            for vertex in 0..<grid.vertexCount {
                let difference = product[vertex] - rhs[vertex]
                residual += difference * difference
            }
            for column in 0..<3 {
                XCTAssertLessThan(residual[column], 1e-11)
            }
        }
    }

    func testSolverReturnsExactSolutionUnchanged() throws {
        let grid = Self.grid(width: 12, height: 9)
        let closenessWeight = 0.002
        let exact = (0..<grid.vertexCount).map { SIMD3<Double>(Double($0), Double($0 % 7), 0.5) }
        let rhs = Self.applySystem(exact, triangles: grid.triangles, closenessWeight: closenessWeight)

        let solution = try VertexUpdate.solveForTesting(
            triangles: grid.triangles,
            vertexCount: grid.vertexCount,
            closenessWeight: closenessWeight,
            rhs: rhs,
            initialGuess: exact
        )

        for (got, want) in zip(solution, exact) {
            XCTAssertEqual(got.x, want.x, accuracy: 1e-9)
            XCTAssertEqual(got.y, want.y, accuracy: 1e-9)
            XCTAssertEqual(got.z, want.z, accuracy: 1e-9)
        }
    }

    /// Reference (A^T A + wI) x as a serial per-face scatter.
    private static func applySystem(
        _ vector: [SIMD3<Double>],
        triangles: [SIMD3<UInt32>],
        closenessWeight: Double
    ) -> [SIMD3<Double>] {
        var result = vector.map { $0 * closenessWeight }
        for triangle in triangles {
            let a = Int(triangle.x), b = Int(triangle.y), c = Int(triangle.z)
            let mean = (vector[a] + vector[b] + vector[c]) / 3.0
            result[a] += vector[a] - mean
            result[b] += vector[b] - mean
            result[c] += vector[c] - mean
        }
        return result
    }

    private static func grid(width: Int, height: Int) -> (vertexCount: Int, triangles: [SIMD3<UInt32>]) {
        var triangles: [SIMD3<UInt32>] = []
        for row in 0..<(height - 1) {
            for column in 0..<(width - 1) {
                let v00 = UInt32(row * width + column)
                let v10 = v00 + 1
                let v01 = v00 + UInt32(width)
                let v11 = v01 + 1
                triangles.append(SIMD3(v00, v10, v11))
                triangles.append(SIMD3(v00, v11, v01))
            }
        }
        return (width * height, triangles)
    }
}
