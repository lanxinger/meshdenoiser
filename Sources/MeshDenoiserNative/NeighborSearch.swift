import Foundation

struct NeighborPair: Sendable, Equatable {
    var face0: UInt32
    var face1: UInt32
    var distance: Float
}

enum NeighborSearch {
    private struct CellKey: Hashable {
        var x: Int
        var y: Int
        var z: Int
    }

    static func findPairs(centroids: [SIMD3<Float>], radius: Float) throws -> [NeighborPair] {
        var pairs = [NeighborPair]()
        pairs.reserveCapacity(centroids.count * 12)
        try forEachPair(centroids: centroids, radius: radius) { pair in
            pairs.append(pair)
        }
        return pairs
    }

    @discardableResult
    static func forEachPair(
        centroids: [SIMD3<Float>],
        radius: Float,
        _ body: (NeighborPair) throws -> Void
    ) throws -> Int {
        guard !centroids.isEmpty, radius.isFinite, radius > 0 else {
            throw NativeDenoiseError.invalidInput
        }

        let radiusSquared = radius * radius
        var minCentroid = SIMD3<Float>(repeating: .infinity)
        for centroid in centroids {
            minCentroid.x = min(minCentroid.x, centroid.x)
            minCentroid.y = min(minCentroid.y, centroid.y)
            minCentroid.z = min(minCentroid.z, centroid.z)
        }

        func cell(for point: SIMD3<Float>) -> CellKey {
            let shifted = (point - minCentroid) / radius
            return CellKey(
                x: Int(floor(Double(shifted.x))),
                y: Int(floor(Double(shifted.y))),
                z: Int(floor(Double(shifted.z)))
            )
        }

        var cells = [CellKey: [Int]]()
        cells.reserveCapacity(centroids.count)
        for (faceIndex, centroid) in centroids.enumerated() {
            cells[cell(for: centroid), default: []].append(faceIndex)
        }

        var pairCount = 0

        for (face0, centroid0) in centroids.enumerated() {
            let baseCell = cell(for: centroid0)
            for dx in -1...1 {
                for dy in -1...1 {
                    for dz in -1...1 {
                        let neighborCell = CellKey(
                            x: baseCell.x + dx,
                            y: baseCell.y + dy,
                            z: baseCell.z + dz
                        )
                        guard let bucket = cells[neighborCell] else { continue }
                        for face1 in bucket where face1 > face0 {
                            let diff = centroid0 - centroids[face1]
                            let distanceSquared = lengthSquared(diff)
                            if distanceSquared < radiusSquared {
                                try body(NeighborPair(
                                    face0: UInt32(face0),
                                    face1: UInt32(face1),
                                    distance: distanceSquared.squareRoot()
                                ))
                                pairCount += 1
                            }
                        }
                    }
                }
            }
        }

        guard pairCount > 0 else { throw NativeDenoiseError.invalidInput }
        return pairCount
    }
}
