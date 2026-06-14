public struct MeshPreprocessOptions: Sendable, Equatable {
    public var weldEpsilon: Float

    public static func conservative(weldEpsilon: Float = 1e-6) -> MeshPreprocessOptions {
        MeshPreprocessOptions(weldEpsilon: weldEpsilon)
    }

    public init(weldEpsilon: Float) {
        self.weldEpsilon = weldEpsilon
    }
}

public struct MeshPreprocessDiagnostics: Sendable, Equatable {
    public var verticesBefore: Int
    public var verticesAfter: Int
    public var facesBefore: Int
    public var facesAfter: Int
    public var boundaryEdgesBefore: Int
    public var boundaryEdgesAfter: Int
    public var nonManifoldEdgesBefore: Int
    public var nonManifoldEdgesAfter: Int
    public var degenerateFacesBefore: Int
    public var degenerateFacesAfter: Int
    public var removedDegenerateFaces: Int
    public var removedDuplicateFaces: Int
    public var removedNonManifoldFaces: Int
    public var removedUnreferencedVertices: Int
}

public struct MeshPreprocessResult: Sendable, Equatable {
    public var positions: [SIMD3<Float>]
    public var indices: [UInt32]
    public var diagnostics: MeshPreprocessDiagnostics
}

public enum MeshPreprocessor {
    public static func repairForDenoising(
        positions: [SIMD3<Float>],
        indices: [UInt32],
        options: MeshPreprocessOptions = .conservative()
    ) throws -> MeshPreprocessResult {
        guard !positions.isEmpty, !indices.isEmpty, indices.count.isMultiple(of: 3), options.weldEpsilon > 0 else {
            throw MeshDenoiseError.invalidInput
        }
        for position in positions {
            guard position.x.isFinite, position.y.isFinite, position.z.isFinite else {
                throw MeshDenoiseError.invalidInput
            }
        }

        let before = meshStats(positions: positions, indices: indices)
        let welded = try weldVertices(positions: positions, indices: indices, epsilon: options.weldEpsilon)
        let filtered = filterFaces(positions: welded.positions, indices: welded.indices)
        let manifold = dropExtraNonManifoldFaces(indices: filtered.indices)
        let oriented = orientFacesConsistently(indices: manifold.indices)
        let compacted = compactVertices(positions: welded.positions, indices: oriented)

        guard !compacted.positions.isEmpty, !compacted.indices.isEmpty else {
            throw MeshDenoiseError.invalidInput
        }
        let after = meshStats(positions: compacted.positions, indices: compacted.indices)
        guard after.degenerateFaces == 0,
              after.nonManifoldEdges == 0,
              !hasDuplicateDirectedEdges(compacted.indices) else {
            throw MeshDenoiseError.invalidInput
        }

        let diagnostics = MeshPreprocessDiagnostics(
            verticesBefore: positions.count,
            verticesAfter: compacted.positions.count,
            facesBefore: indices.count / 3,
            facesAfter: compacted.indices.count / 3,
            boundaryEdgesBefore: before.boundaryEdges,
            boundaryEdgesAfter: after.boundaryEdges,
            nonManifoldEdgesBefore: before.nonManifoldEdges,
            nonManifoldEdgesAfter: after.nonManifoldEdges,
            degenerateFacesBefore: before.degenerateFaces,
            degenerateFacesAfter: after.degenerateFaces,
            removedDegenerateFaces: filtered.removedDegenerateFaces,
            removedDuplicateFaces: filtered.removedDuplicateFaces,
            removedNonManifoldFaces: manifold.removedFaces,
            removedUnreferencedVertices: positions.count - compacted.positions.count
        )

        return MeshPreprocessResult(
            positions: compacted.positions,
            indices: compacted.indices,
            diagnostics: diagnostics
        )
    }

    static func preservedTopologyDiagnostics(
        positions: [SIMD3<Float>],
        indices: [UInt32],
        options: MeshPreprocessOptions = .conservative()
    ) throws -> MeshPreprocessDiagnostics {
        guard !positions.isEmpty, !indices.isEmpty, indices.count.isMultiple(of: 3), options.weldEpsilon > 0 else {
            throw MeshDenoiseError.invalidInput
        }
        for position in positions {
            guard position.x.isFinite, position.y.isFinite, position.z.isFinite else {
                throw MeshDenoiseError.invalidInput
            }
        }

        var referencedVertices = Set<UInt32>()
        referencedVertices.reserveCapacity(positions.count)
        for index in indices {
            guard index < positions.count else { throw MeshDenoiseError.invalidInput }
            referencedVertices.insert(index)
        }

        let stats = meshStats(positions: positions, indices: indices)
        let removedUnreferencedVertices = positions.count - referencedVertices.count
        return MeshPreprocessDiagnostics(
            verticesBefore: positions.count,
            verticesAfter: positions.count - removedUnreferencedVertices,
            facesBefore: indices.count / 3,
            facesAfter: indices.count / 3,
            boundaryEdgesBefore: stats.boundaryEdges,
            boundaryEdgesAfter: stats.boundaryEdges,
            nonManifoldEdgesBefore: stats.nonManifoldEdges,
            nonManifoldEdgesAfter: stats.nonManifoldEdges,
            degenerateFacesBefore: stats.degenerateFaces,
            degenerateFacesAfter: stats.degenerateFaces,
            removedDegenerateFaces: 0,
            removedDuplicateFaces: 0,
            removedNonManifoldFaces: 0,
            removedUnreferencedVertices: removedUnreferencedVertices
        )
    }

    static func hasDuplicateFaces(_ indices: [UInt32]) -> Bool {
        var seenFaces = Set<FaceKey>()
        for offset in stride(from: 0, to: indices.count, by: 3) {
            let a = indices[offset]
            let b = indices[offset + 1]
            let c = indices[offset + 2]
            if !seenFaces.insert(FaceKey(a, b, c)).inserted {
                return true
            }
        }
        return false
    }

    private static func weldVertices(
        positions: [SIMD3<Float>],
        indices: [UInt32],
        epsilon: Float
    ) throws -> (positions: [SIMD3<Float>], indices: [UInt32]) {
        var keyToNewIndex = [PositionKey: UInt32]()
        var oldToNew = Array(repeating: UInt32.max, count: positions.count)
        var weldedPositions = [SIMD3<Float>]()
        weldedPositions.reserveCapacity(positions.count)

        for (oldIndex, position) in positions.enumerated() {
            let key = PositionKey(position: position, epsilon: epsilon)
            if let newIndex = keyToNewIndex[key] {
                oldToNew[oldIndex] = newIndex
            } else {
                let newIndex = UInt32(weldedPositions.count)
                keyToNewIndex[key] = newIndex
                oldToNew[oldIndex] = newIndex
                weldedPositions.append(position)
            }
        }

        var weldedIndices = [UInt32]()
        weldedIndices.reserveCapacity(indices.count)
        for index in indices {
            guard index < positions.count else { throw MeshDenoiseError.invalidInput }
            weldedIndices.append(oldToNew[Int(index)])
        }

        return (weldedPositions, weldedIndices)
    }

    private static func filterFaces(
        positions: [SIMD3<Float>],
        indices: [UInt32]
    ) -> (indices: [UInt32], removedDegenerateFaces: Int, removedDuplicateFaces: Int) {
        var seenFaces = Set<FaceKey>()
        var filtered = [UInt32]()
        filtered.reserveCapacity(indices.count)
        var removedDegenerateFaces = 0
        var removedDuplicateFaces = 0

        for offset in stride(from: 0, to: indices.count, by: 3) {
            let a = indices[offset]
            let b = indices[offset + 1]
            let c = indices[offset + 2]
            guard !isDegenerateFace(a, b, c, positions: positions) else {
                removedDegenerateFaces += 1
                continue
            }

            let key = FaceKey(a, b, c)
            guard seenFaces.insert(key).inserted else {
                removedDuplicateFaces += 1
                continue
            }

            filtered += [a, b, c]
        }

        return (filtered, removedDegenerateFaces, removedDuplicateFaces)
    }

    private static func dropExtraNonManifoldFaces(indices: [UInt32]) -> (indices: [UInt32], removedFaces: Int) {
        let faceCount = indices.count / 3
        var facesByEdge = [MeshEdgeKey: [Int]]()
        for faceIndex in 0..<faceCount {
            let offset = faceIndex * 3
            let a = indices[offset]
            let b = indices[offset + 1]
            let c = indices[offset + 2]
            facesByEdge[MeshEdgeKey(a, b), default: []].append(faceIndex)
            facesByEdge[MeshEdgeKey(b, c), default: []].append(faceIndex)
            facesByEdge[MeshEdgeKey(c, a), default: []].append(faceIndex)
        }

        var removedFaces = Set<Int>()
        for faces in facesByEdge.values where faces.count > 2 {
            for face in faces.dropFirst(2) {
                removedFaces.insert(face)
            }
        }

        var filtered = [UInt32]()
        filtered.reserveCapacity(indices.count)
        for faceIndex in 0..<faceCount where !removedFaces.contains(faceIndex) {
            let offset = faceIndex * 3
            filtered += [indices[offset], indices[offset + 1], indices[offset + 2]]
        }

        return (filtered, removedFaces.count)
    }

    private static func orientFacesConsistently(indices: [UInt32]) -> [UInt32] {
        var triangles = [Triangle]()
        triangles.reserveCapacity(indices.count / 3)
        for offset in stride(from: 0, to: indices.count, by: 3) {
            triangles.append(Triangle(
                a: indices[offset],
                b: indices[offset + 1],
                c: indices[offset + 2]
            ))
        }

        var facesByEdge = [MeshEdgeKey: [Int]]()
        for (faceIndex, triangle) in triangles.enumerated() {
            for edge in triangle.undirectedEdges {
                facesByEdge[edge, default: []].append(faceIndex)
            }
        }

        var visited = Array(repeating: false, count: triangles.count)
        for start in triangles.indices where !visited[start] {
            visited[start] = true
            var queue = [start]
            var readIndex = 0

            while readIndex < queue.count {
                let faceIndex = queue[readIndex]
                readIndex += 1

                for edge in triangles[faceIndex].undirectedEdges {
                    guard let neighbors = facesByEdge[edge] else { continue }
                    for neighbor in neighbors where neighbor != faceIndex && !visited[neighbor] {
                        if triangles[faceIndex].direction(for: edge) == triangles[neighbor].direction(for: edge) {
                            triangles[neighbor].flip()
                        }
                        visited[neighbor] = true
                        queue.append(neighbor)
                    }
                }
            }
        }

        var oriented = [UInt32]()
        oriented.reserveCapacity(indices.count)
        for triangle in triangles {
            oriented += [triangle.a, triangle.b, triangle.c]
        }
        return oriented
    }

    static func hasDuplicateDirectedEdges(_ indices: [UInt32]) -> Bool {
        var seen = Set<DirectedMeshEdge>()
        for offset in stride(from: 0, to: indices.count, by: 3) {
            let a = indices[offset]
            let b = indices[offset + 1]
            let c = indices[offset + 2]
            for edge in [DirectedMeshEdge(a, b), DirectedMeshEdge(b, c), DirectedMeshEdge(c, a)] {
                if !seen.insert(edge).inserted {
                    return true
                }
            }
        }
        return false
    }

    private static func compactVertices(
        positions: [SIMD3<Float>],
        indices: [UInt32]
    ) -> (positions: [SIMD3<Float>], indices: [UInt32]) {
        var oldToNew = [UInt32: UInt32]()
        var compactPositions = [SIMD3<Float>]()
        var compactIndices = [UInt32]()
        compactIndices.reserveCapacity(indices.count)

        for index in indices {
            if let newIndex = oldToNew[index] {
                compactIndices.append(newIndex)
            } else {
                let newIndex = UInt32(compactPositions.count)
                oldToNew[index] = newIndex
                compactIndices.append(newIndex)
                compactPositions.append(positions[Int(index)])
            }
        }

        return (compactPositions, compactIndices)
    }

    private static func meshStats(positions: [SIMD3<Float>], indices: [UInt32]) -> MeshStats {
        var edgeCounts = [MeshEdgeKey: Int]()
        var degenerateFaces = 0

        for offset in stride(from: 0, to: indices.count, by: 3) {
            let a = indices[offset]
            let b = indices[offset + 1]
            let c = indices[offset + 2]
            if a >= positions.count || b >= positions.count || c >= positions.count
                || isDegenerateFace(a, b, c, positions: positions) {
                degenerateFaces += 1
                continue
            }

            edgeCounts[MeshEdgeKey(a, b), default: 0] += 1
            edgeCounts[MeshEdgeKey(b, c), default: 0] += 1
            edgeCounts[MeshEdgeKey(c, a), default: 0] += 1
        }

        return MeshStats(
            boundaryEdges: edgeCounts.values.filter { $0 == 1 }.count,
            nonManifoldEdges: edgeCounts.values.filter { $0 > 2 }.count,
            degenerateFaces: degenerateFaces
        )
    }

    private static func isDegenerateFace(
        _ a: UInt32,
        _ b: UInt32,
        _ c: UInt32,
        positions: [SIMD3<Float>]
    ) -> Bool {
        guard a != b, b != c, a != c else { return true }
        let p0 = positions[Int(a)]
        let p1 = positions[Int(b)]
        let p2 = positions[Int(c)]
        let areaVector = cross(p1 - p0, p2 - p0)
        return lengthSquared(areaVector) == 0
    }

    private static func cross(_ lhs: SIMD3<Float>, _ rhs: SIMD3<Float>) -> SIMD3<Float> {
        SIMD3(
            lhs.y * rhs.z - lhs.z * rhs.y,
            lhs.z * rhs.x - lhs.x * rhs.z,
            lhs.x * rhs.y - lhs.y * rhs.x
        )
    }

    private static func lengthSquared(_ vector: SIMD3<Float>) -> Float {
        vector.x * vector.x + vector.y * vector.y + vector.z * vector.z
    }
}

private struct MeshStats {
    var boundaryEdges: Int
    var nonManifoldEdges: Int
    var degenerateFaces: Int
}

private struct Triangle {
    var a: UInt32
    var b: UInt32
    var c: UInt32

    var undirectedEdges: [MeshEdgeKey] {
        [
            MeshEdgeKey(a, b),
            MeshEdgeKey(b, c),
            MeshEdgeKey(c, a),
        ]
    }

    mutating func flip() {
        swap(&b, &c)
    }

    func direction(for edge: MeshEdgeKey) -> Int {
        if hasDirectedEdge(edge.a, edge.b) {
            return 1
        }
        if hasDirectedEdge(edge.b, edge.a) {
            return -1
        }
        return 0
    }

    private func hasDirectedEdge(_ first: UInt32, _ second: UInt32) -> Bool {
        (a == first && b == second)
            || (b == first && c == second)
            || (c == first && a == second)
    }
}

private struct PositionKey: Hashable {
    var x: Int64
    var y: Int64
    var z: Int64

    init(position: SIMD3<Float>, epsilon: Float) {
        x = Int64((position.x / epsilon).rounded())
        y = Int64((position.y / epsilon).rounded())
        z = Int64((position.z / epsilon).rounded())
    }
}

private struct FaceKey: Hashable {
    var a: UInt32
    var b: UInt32
    var c: UInt32

    init(_ first: UInt32, _ second: UInt32, _ third: UInt32) {
        let sorted = [first, second, third].sorted()
        a = sorted[0]
        b = sorted[1]
        c = sorted[2]
    }
}

private struct MeshEdgeKey: Hashable {
    var a: UInt32
    var b: UInt32

    init(_ first: UInt32, _ second: UInt32) {
        if first < second {
            a = first
            b = second
        } else {
            a = second
            b = first
        }
    }
}

private struct DirectedMeshEdge: Hashable {
    var a: UInt32
    var b: UInt32

    init(_ first: UInt32, _ second: UInt32) {
        a = first
        b = second
    }
}
