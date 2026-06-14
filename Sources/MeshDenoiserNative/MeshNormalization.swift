struct MeshNormalization: Sendable {
    var originalCenter: SIMD3<Double>
    var originalScale: Double

    static func normalize(_ positions: [SIMD3<Float>]) throws -> (positions: [SIMD3<Float>], transform: MeshNormalization) {
        guard !positions.isEmpty else { throw NativeDenoiseError.invalidInput }

        let center = positions.reduce(SIMD3<Double>(repeating: 0)) { partial, position in
            partial + SIMD3<Double>(Double(position.x), Double(position.y), Double(position.z))
        } / Double(positions.count)

        var minPoint = SIMD3<Double>(repeating: .infinity)
        var maxPoint = SIMD3<Double>(repeating: -.infinity)
        for position in positions {
            let point = SIMD3<Double>(Double(position.x), Double(position.y), Double(position.z))
            minPoint.x = Swift.min(minPoint.x, point.x)
            minPoint.y = Swift.min(minPoint.y, point.y)
            minPoint.z = Swift.min(minPoint.z, point.z)
            maxPoint.x = Swift.max(maxPoint.x, point.x)
            maxPoint.y = Swift.max(maxPoint.y, point.y)
            maxPoint.z = Swift.max(maxPoint.z, point.z)
        }

        let dimension = maxPoint - minPoint
        let scale = (dimension.x * dimension.x + dimension.y * dimension.y + dimension.z * dimension.z).squareRoot()
        guard scale.isFinite, scale > 0 else { throw NativeDenoiseError.invalidInput }

        let normalized = positions.map { position -> SIMD3<Float> in
            let point = (SIMD3<Double>(Double(position.x), Double(position.y), Double(position.z)) - center) / scale
            return SIMD3<Float>(Float(point.x), Float(point.y), Float(point.z))
        }

        return (
            normalized,
            MeshNormalization(originalCenter: center, originalScale: scale)
        )
    }

    func restore(_ positions: [SIMD3<Float>]) -> [SIMD3<Float>] {
        guard !positions.isEmpty else { return [] }

        let currentCenter = positions.reduce(SIMD3<Double>(repeating: 0)) { partial, position in
            partial + SIMD3<Double>(Double(position.x), Double(position.y), Double(position.z))
        } / Double(positions.count)

        return positions.map { position in
            let point = (SIMD3<Double>(Double(position.x), Double(position.y), Double(position.z)) - currentCenter)
                * originalScale + originalCenter
            return SIMD3<Float>(Float(point.x), Float(point.y), Float(point.z))
        }
    }
}
