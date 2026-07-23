import Foundation

struct NeighborPair: Sendable, Equatable {
    var face0: UInt32
    var face1: UInt32
    /// Kept squared because filtering consumes squared distance directly.
    var distanceSquared: Float
}

enum NeighborSearch {
    struct Grid: Sendable {
        private struct CellKey: Hashable, Sendable {
            var x: Int
            var y: Int
            var z: Int
        }

        private let centroids: [SIMD3<Float>]
        private let radius: Float
        private let radiusSquared: Float
        private let minCentroid: SIMD3<Float>
        private let cells: [CellKey: [Int]]

        init(
            centroids: [SIMD3<Float>],
            radius: Float,
            shouldCancel: (@Sendable () -> Bool)? = nil
        ) throws {
            let radiusSquared = radius * radius
            guard !centroids.isEmpty,
                  centroids.count <= UInt32.max,
                  radius.isFinite,
                  radius > 0,
                  radiusSquared.isFinite,
                  radiusSquared > 0
            else {
                throw NativeDenoiseError.invalidInput
            }

            var minCentroid = SIMD3<Float>(repeating: .infinity)
            for (faceIndex, centroid) in centroids.enumerated() {
                if faceIndex.isMultiple(of: 1_024), shouldCancel?() == true {
                    throw NativeDenoiseError.cancelled
                }
                guard centroid.x.isFinite, centroid.y.isFinite, centroid.z.isFinite else {
                    throw NativeDenoiseError.invalidInput
                }
                minCentroid.x = min(minCentroid.x, centroid.x)
                minCentroid.y = min(minCentroid.y, centroid.y)
                minCentroid.z = min(minCentroid.z, centroid.z)
            }

            var cells = [CellKey: [Int]]()
            cells.reserveCapacity(centroids.count)
            for (faceIndex, centroid) in centroids.enumerated() {
                if faceIndex.isMultiple(of: 1_024), shouldCancel?() == true {
                    throw NativeDenoiseError.cancelled
                }
                let key = try Self.cell(for: centroid, minimum: minCentroid, radius: radius)
                cells[key, default: []].append(faceIndex)
            }

            self.centroids = centroids
            self.radius = radius
            self.radiusSquared = radiusSquared
            self.minCentroid = minCentroid
            self.cells = cells
        }

        @discardableResult
        func forEachPair(
            shouldCancel: (@Sendable () -> Bool)? = nil,
            _ body: (NeighborPair) throws -> Void
        ) throws -> Int {
            var pairCount = 0
            var candidateCount = 0

            for (face0, centroid0) in centroids.enumerated() {
                if face0.isMultiple(of: 1_024), shouldCancel?() == true {
                    throw NativeDenoiseError.cancelled
                }
                let baseCell = try Self.cell(for: centroid0, minimum: minCentroid, radius: radius)
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
                                candidateCount += 1
                                if candidateCount.isMultiple(of: 4_096), shouldCancel?() == true {
                                    throw NativeDenoiseError.cancelled
                                }
                                let diff = centroid0 - centroids[face1]
                                let distanceSquared = lengthSquared(diff)
                                if distanceSquared < radiusSquared {
                                    try body(NeighborPair(
                                        face0: UInt32(face0),
                                        face1: UInt32(face1),
                                        distanceSquared: distanceSquared
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

        private static func cell(
            for point: SIMD3<Float>,
            minimum: SIMD3<Float>,
            radius: Float
        ) throws -> CellKey {
            let shifted = (point - minimum) / radius
            guard let x = Int(exactly: floor(Double(shifted.x))),
                  let y = Int(exactly: floor(Double(shifted.y))),
                  let z = Int(exactly: floor(Double(shifted.z))),
                  x > Int.min, x < Int.max,
                  y > Int.min, y < Int.max,
                  z > Int.min, z < Int.max
            else {
                throw NativeDenoiseError.invalidInput
            }
            return CellKey(x: x, y: y, z: z)
        }
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
        shouldCancel: (@Sendable () -> Bool)? = nil,
        _ body: (NeighborPair) throws -> Void
    ) throws -> Int {
        let grid = try Grid(centroids: centroids, radius: radius, shouldCancel: shouldCancel)
        return try grid.forEachPair(shouldCancel: shouldCancel, body)
    }
}
