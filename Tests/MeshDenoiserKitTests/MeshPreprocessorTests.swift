import XCTest
@testable import MeshDenoiserKit
@testable import MeshDenoiserNative

final class MeshPreprocessorTests: XCTestCase {

    func testConservativeRepairWeldsVerticesAndRemovesInvalidFaces() throws {
        let positions: [SIMD3<Float>] = [
            [0, 0, 0],
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 0.0000001],
            [2, 2, 2],
        ]
        let indices: [UInt32] = [
            0, 1, 2,
            3, 1, 2,
            0, 0, 1,
        ]

        let repaired = try MeshPreprocessor.repairForDenoising(
            positions: positions,
            indices: indices,
            options: .conservative(weldEpsilon: 0.000001)
        )

        XCTAssertEqual(repaired.positions.count, 3)
        XCTAssertEqual(repaired.indices, [0, 1, 2])
        XCTAssertEqual(repaired.diagnostics.removedDegenerateFaces, 1)
        XCTAssertEqual(repaired.diagnostics.removedDuplicateFaces, 1)
        XCTAssertEqual(repaired.diagnostics.removedUnreferencedVertices, 2)
    }

    func testConservativeRepairDropsExtraFacesOnNonManifoldEdges() throws {
        let positions: [SIMD3<Float>] = [
            [0, 0, 0],
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 1],
            [0, 0, -1],
        ]
        let indices: [UInt32] = [
            0, 1, 2,
            1, 0, 3,
            0, 1, 4,
        ]

        let repaired = try MeshPreprocessor.repairForDenoising(
            positions: positions,
            indices: indices,
            options: .conservative()
        )

        let mesh = try MeshValidation.makeMesh(positions: repaired.positions, indices: repaired.indices)
        XCTAssertNoThrow(try MeshConnectivity.build(mesh: mesh))
        XCTAssertEqual(repaired.indices.count / 3, 2)
        XCTAssertEqual(repaired.diagnostics.nonManifoldEdgesBefore, 1)
        XCTAssertEqual(repaired.diagnostics.nonManifoldEdgesAfter, 0)
        XCTAssertEqual(repaired.diagnostics.removedNonManifoldFaces, 1)
    }

    func testConservativeRepairOrientsSharedEdgesOppositely() throws {
        let positions: [SIMD3<Float>] = [
            [0, 0, 0],
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 1],
        ]
        let indices: [UInt32] = [
            0, 1, 2,
            0, 1, 3,
        ]

        let repaired = try MeshPreprocessor.repairForDenoising(
            positions: positions,
            indices: indices,
            options: .conservative()
        )

        XCTAssertFalse(hasDuplicateDirectedEdges(repaired.indices))
    }

    func testRepairRejectsMeshesWithNoRemainingFaces() {
        let positions: [SIMD3<Float>] = [
            [0, 0, 0],
            [1, 0, 0],
            [2, 0, 0],
        ]
        let indices: [UInt32] = [0, 1, 2]

        XCTAssertThrowsError(try MeshPreprocessor.repairForDenoising(
            positions: positions,
            indices: indices,
            options: .conservative()
        )) { error in
            XCTAssertEqual(error as? MeshDenoiseError, .invalidInput)
        }
    }

    private func hasDuplicateDirectedEdges(_ indices: [UInt32]) -> Bool {
        var seen = Set<DirectedEdge>()
        for offset in stride(from: 0, to: indices.count, by: 3) {
            let a = indices[offset]
            let b = indices[offset + 1]
            let c = indices[offset + 2]
            for edge in [DirectedEdge(a, b), DirectedEdge(b, c), DirectedEdge(c, a)] {
                if !seen.insert(edge).inserted {
                    return true
                }
            }
        }
        return false
    }
}

private struct DirectedEdge: Hashable {
    var a: UInt32
    var b: UInt32

    init(_ a: UInt32, _ b: UInt32) {
        self.a = a
        self.b = b
    }
}
