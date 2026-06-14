enum GuidanceNormals {
    static func compute(connectivity: MeshConnectivity.Result) -> [SIMD3<Float>] {
        let faceCount = connectivity.faceGeometry.normals.count
        var edgeSaliency = [Float](repeating: 0, count: connectivity.edges.count)

        for (edgeIndex, edge) in connectivity.edges.enumerated() {
            guard let face1 = edge.face1 else { continue }
            let diff = connectivity.faceGeometry.normals[Int(edge.face0)]
                - connectivity.faceGeometry.normals[Int(face1)]
            edgeSaliency[edgeIndex] = length(diff)
        }

        var patchNormalConsistency = [Float](repeating: 0, count: faceCount)
        var patchAverageNormal = [SIMD3<Float>](repeating: .zero, count: faceCount)
        var neighborhoodFacesPerFace = Array(repeating: [UInt32](), count: faceCount)
        var faceStamp = [Int](repeating: -1, count: faceCount)
        var edgeStamp = [Int](repeating: -1, count: connectivity.edges.count)

        for faceIndex in 0..<faceCount {
            let facesInPatch = collectPatchFaces(
                faceIndex: faceIndex,
                connectivity: connectivity,
                stamp: &faceStamp
            )
            let edgesInPatch = collectPatchEdges(
                faceIndex: faceIndex,
                connectivity: connectivity,
                stamp: &edgeStamp
            )
            neighborhoodFacesPerFace[faceIndex] = facesInPatch

            var sumSaliency: Float = 0
            var maxSaliency: Float = 0
            for edgeIndex in edgesInPatch {
                let value = edgeSaliency[Int(edgeIndex)]
                sumSaliency += value
                maxSaliency = max(maxSaliency, value)
            }

            var maxNormalDiff: Float = 0
            if facesInPatch.count > 1 {
                for lhsIndex in 0..<(facesInPatch.count - 1) {
                    for rhsIndex in (lhsIndex + 1)..<facesInPatch.count {
                        let lhs = connectivity.faceGeometry.normals[Int(facesInPatch[lhsIndex])]
                        let rhs = connectivity.faceGeometry.normals[Int(facesInPatch[rhsIndex])]
                        maxNormalDiff = max(maxNormalDiff, length(lhs - rhs))
                    }
                }
            }

            patchNormalConsistency[faceIndex] = (maxSaliency / (1e-9 + sumSaliency)) * maxNormalDiff

            var average = SIMD3<Float>.zero
            for face in facesInPatch {
                average += connectivity.faceGeometry.normals[Int(face)]
                    * connectivity.faceGeometry.areaWeights[Int(face)]
            }
            patchAverageNormal[faceIndex] = normalized(average, fallback: connectivity.faceGeometry.normals[faceIndex])
        }

        return (0..<faceCount).map { faceIndex in
            let candidates = neighborhoodFacesPerFace[faceIndex]
            guard let best = candidates.min(by: {
                patchNormalConsistency[Int($0)] < patchNormalConsistency[Int($1)]
            }) else {
                return connectivity.faceGeometry.normals[faceIndex]
            }
            return patchAverageNormal[Int(best)]
        }
    }

    private static func collectPatchFaces(
        faceIndex: Int,
        connectivity: MeshConnectivity.Result,
        stamp: inout [Int]
    ) -> [UInt32] {
        let triangle = connectivity.triangles[faceIndex]
        var faces = [UInt32]()
        faces.reserveCapacity(12)

        for vertex in [triangle.x, triangle.y, triangle.z] {
            for offset in connectivity.vertexToFaces.range(for: Int(vertex)) {
                let face = connectivity.vertexToFaces.values[offset]
                if stamp[Int(face)] != faceIndex {
                    stamp[Int(face)] = faceIndex
                    faces.append(face)
                }
            }
        }

        return faces
    }

    private static func collectPatchEdges(
        faceIndex: Int,
        connectivity: MeshConnectivity.Result,
        stamp: inout [Int]
    ) -> [UInt32] {
        let triangle = connectivity.triangles[faceIndex]
        var edges = [UInt32]()
        edges.reserveCapacity(12)

        for vertex in [triangle.x, triangle.y, triangle.z] {
            for offset in connectivity.vertexToNonboundaryEdges.range(for: Int(vertex)) {
                let edge = connectivity.vertexToNonboundaryEdges.values[offset]
                if stamp[Int(edge)] != faceIndex {
                    stamp[Int(edge)] = faceIndex
                    edges.append(edge)
                }
            }
        }

        return edges
    }
}

func normalized(_ vector: SIMD3<Float>, fallback: SIMD3<Float>) -> SIMD3<Float> {
    let magnitude = length(vector)
    guard magnitude.isFinite, magnitude > 0 else { return fallback }
    return vector / magnitude
}
