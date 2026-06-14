enum MeshValidation {
    static func makeMesh(positions: [SIMD3<Float>], indices: [UInt32]) throws -> NativeMesh {
        guard !positions.isEmpty, !indices.isEmpty, indices.count.isMultiple(of: 3) else {
            throw NativeDenoiseError.invalidInput
        }

        for position in positions {
            guard position.x.isFinite, position.y.isFinite, position.z.isFinite else {
                throw NativeDenoiseError.invalidInput
            }
        }

        let vertexCount = UInt32(positions.count)
        var triangles = [SIMD3<UInt32>]()
        triangles.reserveCapacity(indices.count / 3)

        for offset in stride(from: 0, to: indices.count, by: 3) {
            let triangle = SIMD3(indices[offset], indices[offset + 1], indices[offset + 2])
            guard triangle.x < vertexCount, triangle.y < vertexCount, triangle.z < vertexCount else {
                throw NativeDenoiseError.invalidInput
            }
            guard triangle.x != triangle.y, triangle.y != triangle.z, triangle.x != triangle.z else {
                throw NativeDenoiseError.invalidInput
            }
            triangles.append(triangle)
        }

        return NativeMesh(positions: positions, triangles: triangles)
    }
}
